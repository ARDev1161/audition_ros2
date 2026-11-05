/* Mermaid SVG modal with pointer-centered wheel/pinch zoom and drag-to-pan.
 * Works with asynchronously rendered sphinxcontrib-mermaid diagrams.
 * No third-party runtime or image conversion required.
 */
(() => {
  "use strict";
  // Support both Sphinx Mermaid wrappers and directly rendered Mermaid SVGs.
  const SELECTOR = [
    ".mermaid-diagram svg", ".mermaid-container svg", ".mermaid svg",
    "svg[id^='mermaid-']", "svg[aria-roledescription='flowchart']"
  ].join(", ");
  const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));
  let overlay, viewport, stage, zoomLabel, closeButton, active;

  function redraw() {
    if (!active) return;
    stage.style.transform = "translate(" + active.x + "px," + active.y + "px) scale(" + active.zoom + ")";
    zoomLabel.textContent = Math.round(active.zoom * 100) + "%";
  }
  function zoomAt(multiplier, px, py) {
    if (!active) return;
    const next = clamp(active.zoom * multiplier, 0.05, 10);
    const ratio = next / active.zoom;
    active.x = px - (px - active.x) * ratio;
    active.y = py - (py - active.y) * ratio;
    active.zoom = next;
    redraw();
  }
  function fit() {
    if (!active) return;
    const zoom = Math.min((viewport.clientWidth - 48) / active.width,
                          (viewport.clientHeight - 48) / active.height);
    active.zoom = clamp(zoom, 0.05, 10);
    active.x = (viewport.clientWidth - active.width * active.zoom) / 2;
    active.y = (viewport.clientHeight - active.height * active.zoom) / 2;
    redraw();
  }
  function close() {
    if (!active) return;
    const old = active;
    active = null;
    for (const id of old.pointers.keys()) {
      if (viewport.hasPointerCapture(id)) viewport.releasePointerCapture(id);
    }
    old.pointers.clear();
    old.placeholder.replaceWith(old.svg);
    if (old.style === null) old.svg.removeAttribute("style");
    else old.svg.setAttribute("style", old.style);
    stage.replaceChildren();
    overlay.hidden = true;
    viewport.classList.remove("diagram-viewer-dragging");
    document.body.style.overflow = old.overflow;
    old.opener.focus({preventScroll: true});
  }
  function button(text, title, fn) {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = text;
    b.title = title;
    b.setAttribute("aria-label", title);
    b.addEventListener("click", fn);
    return b;
  }
  function setup() {
    if (overlay) return;
    overlay = document.createElement("section");
    overlay.className = "diagram-viewer-overlay";
    overlay.setAttribute("role", "dialog");
    overlay.setAttribute("aria-modal", "true");
    overlay.setAttribute("aria-label", "Zoomable Mermaid diagram");
    overlay.hidden = true;
    const toolbar = document.createElement("div");
    toolbar.className = "diagram-viewer-toolbar";
    const minus = button("−", "Zoom out", () => zoomAt(1 / 1.3, viewport.clientWidth / 2, viewport.clientHeight / 2));
    const plus = button("+", "Zoom in", () => zoomAt(1.3, viewport.clientWidth / 2, viewport.clientHeight / 2));
    zoomLabel = document.createElement("output");
    zoomLabel.className = "diagram-viewer-zoom";
    zoomLabel.setAttribute("aria-label", "Zoom level");
    closeButton = button("×", "Close diagram (Escape)", close);
    toolbar.append(minus, zoomLabel, plus, button("Fit", "Fit diagram (0)", fit), closeButton);
    viewport = document.createElement("div");
    viewport.className = "diagram-viewer-viewport";
    viewport.tabIndex = 0;
    viewport.setAttribute("aria-label", "Wheel or pinch to zoom; drag to move; double-click to fit");
    stage = document.createElement("div");
    stage.className = "diagram-viewer-stage";
    viewport.append(stage);
    overlay.append(toolbar, viewport);
    document.body.append(overlay);

    viewport.addEventListener("wheel", (event) => {
      if (!active) return;
      event.preventDefault();
      const rect = viewport.getBoundingClientRect();
      zoomAt(Math.exp(-event.deltaY * 0.002), event.clientX - rect.left, event.clientY - rect.top);
    }, {passive: false});
    // Touch, pen and mouse share viewport-relative pointer coordinates.
    const point = (event) => {
      const rect = viewport.getBoundingClientRect();
      return {x: event.clientX - rect.left, y: event.clientY - rect.top};
    };
    const midpoint = (a, b) => ({x: (a.x + b.x) / 2, y: (a.y + b.y) / 2});
    const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
    viewport.addEventListener("pointerdown", (event) => {
      if (!active || (event.pointerType !== "touch" && event.button !== 0) ||
          event.target.closest("a")) return;
      event.preventDefault();
      active.pointers.set(event.pointerId, point(event));
      viewport.setPointerCapture(event.pointerId);
      viewport.classList.add("diagram-viewer-dragging");
    });
    viewport.addEventListener("pointermove", (event) => {
      if (!active || !active.pointers.has(event.pointerId)) return;
      const previous = active.pointers.get(event.pointerId);
      const current = point(event);
      if (active.pointers.size === 1) {
        // When one finger remains after a pinch, keep panning smoothly.
        active.x += current.x - previous.x;
        active.y += current.y - previous.y;
      } else {
        // Two-finger midpoint changes pan, distance changes zoom.
        const before = Array.from(active.pointers.values()).slice(0, 2);
        const oldCenter = midpoint(before[0], before[1]);
        const oldDistance = distance(before[0], before[1]);
        active.pointers.set(event.pointerId, current);
        const after = Array.from(active.pointers.values()).slice(0, 2);
        const newCenter = midpoint(after[0], after[1]);
        const newDistance = distance(after[0], after[1]);
        const multiplier = oldDistance > 0 ? newDistance / oldDistance : 1;
        const nextZoom = clamp(active.zoom * multiplier, 0.05, 10);
        const ratio = nextZoom / active.zoom;
        active.x = newCenter.x - (oldCenter.x - active.x) * ratio;
        active.y = newCenter.y - (oldCenter.y - active.y) * ratio;
        active.zoom = nextZoom;
      }
      active.pointers.set(event.pointerId, current);
      redraw();
    });
    const endPointer = (event) => {
      if (!active || !active.pointers.delete(event.pointerId)) return;
      if (viewport.hasPointerCapture(event.pointerId)) viewport.releasePointerCapture(event.pointerId);
      if (active.pointers.size === 0) viewport.classList.remove("diagram-viewer-dragging");
    };
    viewport.addEventListener("pointerup", endPointer);
    viewport.addEventListener("pointercancel", endPointer);
    viewport.addEventListener("lostpointercapture", endPointer);
    viewport.addEventListener("dblclick", (event) => {event.preventDefault(); fit();});
    document.addEventListener("keydown", (event) => {
      if (!active) return;
      if (event.key === "Escape") {event.preventDefault(); close();}
      else if (event.key === "+" || event.key === "=") {event.preventDefault(); zoomAt(1.3, viewport.clientWidth / 2, viewport.clientHeight / 2);}
      else if (event.key === "-") {event.preventDefault(); zoomAt(1 / 1.3, viewport.clientWidth / 2, viewport.clientHeight / 2);}
      else if (event.key === "0") {event.preventDefault(); fit();}
      else if (event.key === "Tab") {
        const focusables = Array.from(overlay.querySelectorAll("button, [tabindex='0']"));
        const position = focusables.indexOf(document.activeElement);
        if (event.shiftKey && position <= 0) {event.preventDefault(); focusables[focusables.length - 1].focus();}
        else if (!event.shiftKey && position === focusables.length - 1) {event.preventDefault(); focusables[0].focus();}
      }
    });
    window.addEventListener("resize", () => {if (active) fit();});
  }
  function open(svg, opener) {
    if (active) close();
    setup();
    const box = svg.viewBox && svg.viewBox.baseVal;
    const rect = svg.getBoundingClientRect();
    const width = box && box.width > 0 ? box.width : rect.width;
    const height = box && box.height > 0 ? box.height : rect.height;
    if (!(width > 0 && height > 0)) return;
    const placeholder = document.createComment("original Mermaid position");
    svg.before(placeholder);
    active = {
      svg, placeholder, opener, width, height,
      style: svg.getAttribute("style"), overflow: document.body.style.overflow,
      zoom: 1, x: 0, y: 0, pointers: new Map()
    };
    svg.style.width = width + "px";
    svg.style.height = height + "px";
    svg.style.maxWidth = "none";
    svg.style.maxHeight = "none";
    svg.style.display = "block";
    stage.append(svg);
    overlay.hidden = false;
    document.body.style.overflow = "hidden";
    fit();
    closeButton.focus({preventScroll: true});
  }
  function attach() {
    document.querySelectorAll(SELECTOR).forEach((svg) => {
      // The original SVG is temporarily moved into the viewer while open.
      if (svg.closest(".diagram-viewer-overlay")) return;
      const source = svg.closest(".mermaid-diagram")
        || svg.closest(".mermaid-container")
        || svg.closest(".mermaid")
        || svg.parentElement;
      if (!source || source.dataset.diagramViewerReady === "true") return;
      source.dataset.diagramViewerReady = "true";
      source.classList.add("diagram-viewer-source");
      // Preserve the Mermaid group's existing accessibility label, if present.
      if (!source.hasAttribute("tabindex")) source.tabIndex = 0;
      if (!source.hasAttribute("role")) source.setAttribute("role", "button");
      if (!source.hasAttribute("aria-label")) {
        source.setAttribute("aria-label", "Open diagram in zoomable viewer");
      }
      source.setAttribute("aria-haspopup", "dialog");
      source.title = "Click to expand diagram";

      const launch = (event) => {
        if (event.defaultPrevented) return;
        if (event.type === "click") {
          if (event.button !== 0 || event.ctrlKey || event.metaKey ||
              event.altKey || event.shiftKey) return;
          // Keep clickable nodes and the extension's own controls working.
          if (event.target.closest("a, button, input, textarea, select, summary")) return;
        } else if (event.type === "keydown") {
          // Don't hijack keyboard activation on interactive descendants.
          if (event.target !== source ||
              (event.key !== "Enter" && event.key !== " ")) return;
        } else {
          return;
        }
        const current = source.querySelector(
          "svg[id^='mermaid-'], svg[aria-roledescription='flowchart'], .mermaid svg, svg"
        );
        if (!current) return;
        event.preventDefault();
        open(current, source);
      };
      source.addEventListener("click", launch);
      source.addEventListener("keydown", launch);
    });
  }
  function init() {
    attach();
    let pending = false;
    new MutationObserver(() => {
      if (pending) return;
      pending = true;
      requestAnimationFrame(() => {pending = false; attach();});
    }).observe(document.body, {childList: true, subtree: true});
  }
  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init, {once: true});
  else init();
})();
