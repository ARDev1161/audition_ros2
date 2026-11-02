"""ROS graph smoke: real typed M2c capability service and immutable parameters."""
import time

import rclpy
from audition_msgs.srv import GetCapabilities
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


def invoke(node, client, request):
    if not client.wait_for_service(timeout_sec=20.0):
        print("Discovered ROS nodes:", node.get_node_names_and_namespaces(), flush=True)
        print("Discovered ROS services:", node.get_service_names_and_types(), flush=True)
        raise AssertionError(f"Service unavailable: {client.srv_name}")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done() or future.exception() is not None:
        raise AssertionError(f"Service call failed: {client.srv_name}")
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node("profile_capabilities_ci")
    try:
        caps = invoke(node, node.create_client(
            GetCapabilities, "/audition/semantic_perception/get_capabilities"),
            GetCapabilities.Request())
        assert caps.profile_name == "replay_ci", caps
        assert caps.spatial_backend == "mono", caps
        assert caps.classifier_backend == "disabled", caps
        assert not caps.classifier_active and not caps.speech_active, caps
        assert not caps.runtime_reconfiguration_supported, caps
        assert "disabled" in caps.compiled_classifiers, caps
        assert caps.loaded_models == [], caps

        change = SetParameters.Request()
        change.parameters = [Parameter(
            name="classifier_backend",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value="yamnet"))]
        reply = invoke(node, node.create_client(
            SetParameters, "/audition/semantic_perception/set_parameters"), change)
        assert len(reply.results) == 1 and not reply.results[0].successful, reply
        assert "startup-only" in reply.results[0].reason, reply
        again = invoke(node, node.create_client(
            GetCapabilities, "/audition/semantic_perception/get_capabilities"),
            GetCapabilities.Request())
        assert again.classifier_backend == "disabled"
        print("M2c profile service + immutable parameter contract PASS")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
