// Interactividad del dashboard. Se incrusta al final del <body>, cuando todos
// los <div> de plotly existen ya
//
// Estos dos ids los pone pagina.py al crear los <div>
// Ojo si cambian alli hay que cambiarlos aqui
const ID_GRAFICA_2 = "g-trayectorias";   // seccion 2, trayectorias
const ID_GRAFICA_3 = "g-derrape-bag";    // seccion 3, distancia de derrape

// Indice del paso activo de un slider (0 si no tiene)
function vueltaActiva(gd) {
  const s = (gd.layout.sliders || [])[0];
  return s ? (s.active || 0) : 0;
}

// ---------------------------------------------------------------------------
// Seccion 2: los checkboxes que quitan o ponen cada capa
// ---------------------------------------------------------------------------
// Son checkboxes HTML  
// El slider de vueltas volveria a encender una capa que el checkbox acaba de apagar. 
// Por eso se reaplica el estado de los checkboxes despues de cada cambio de vuelta
//
(function () {
  const gd = document.getElementById(ID_GRAFICA_2);
  const cont = document.getElementById("checks-tray-2");
  if (!gd || !cont) return;

  function marcado(serie) {
    const el = cont.querySelector('input[data-serie="' + serie + '"]');
    return el ? el.checked : true;  
  }

  // Aplica el estado de los checkboxes a la vuelta activa
  function reaplicar() {
    const meta = gd.layout.meta || {};
    const k = vueltaActiva(gd);
    const on = [], off = [];
    function aplica(visible, idx) {
      if (idx === undefined || idx === null || idx < 0) return;
      (visible ? on : off).push(idx);
    }
    aplica(marcado("base"), meta.idx_base);
    aplica(marcado("base"), meta.idx_fantasma_base);
    aplica(marcado("delantera"), (meta.idx_delantera || [])[k]);
    aplica(marcado("delantera"), meta.idx_fantasma_delantera);
    aplica(marcado("trasera"), (meta.idx_trasera || [])[k]);
    aplica(marcado("trasera"), meta.idx_fantasma_trasera);
    aplica(marcado("perp"), (meta.idx_perp || [])[k]);
    aplica(marcado("perp"), meta.idx_fantasma_perp);
    if (off.length) Plotly.restyle(gd, {visible: false}, off);
    if (on.length) Plotly.restyle(gd, {visible: true}, on);
  }

  gd.__reaplicar = reaplicar;   
  cont.querySelectorAll('input[type="checkbox"]').forEach(function (el) {
    el.addEventListener("change", reaplicar);
  });
  // Al cambiar de vuelta, el slider reescribe todas las visibilidades
  gd.on("plotly_sliderchange", function () { setTimeout(reaplicar, 0); });
  reaplicar();
})();

// ---------------------------------------------------------------------------
// Seccion 3: las flechas ← y → cambian de vuelta
// ---------------------------------------------------------------------------
// Solo mientras el raton esta encima de la grafica
(function () {
  const gd = document.getElementById(ID_GRAFICA_3);
  if (!gd) return;
  let encima = false;
  gd.addEventListener("mouseenter", function () { encima = true; });
  gd.addEventListener("mouseleave", function () { encima = false; });
  document.addEventListener("keydown", function (ev) {
    if (!encima) return;
    let paso = 0;
    if (ev.key === "ArrowRight") paso = 1;
    else if (ev.key === "ArrowLeft") paso = -1;
    else return;
    ev.preventDefault();   // que la pagina no se desplace con las flechas
    const slider = gd.layout.sliders[0];
    const actual = vueltaActiva(gd);
    const k = Math.min(slider.steps.length - 1, Math.max(0, actual + paso));
    if (k === actual) return;
    Plotly.update(gd, slider.steps[k].args[0], {"sliders[0].active": k});
  });
})();

// ---------------------------------------------------------------------------
// La vuelta de la seccion 2 y la de la 3 van atadas
// ---------------------------------------------------------------------------
// Las dos usan el numero de vuelta como etiqueta hay que unirlas 
(function () {
  const g2 = document.getElementById(ID_GRAFICA_2);
  const g3 = document.getElementById(ID_GRAFICA_3);
  if (!g2 || !g3) return;
  let sincronizando = false;

  function pasoConEtiqueta(gd, etiqueta) {
    const s = (gd.layout.sliders || [])[0];
    if (!s) return -1;
    for (let i = 0; i < s.steps.length; i++) {
      if (String(s.steps[i].label) === String(etiqueta)) return i;
    }
    return -1;
  }

  function enlazar(origen, destino) {
    origen.on("plotly_sliderchange", function (ev) {
      if (sincronizando || !ev.step) return;
      const k = pasoConEtiqueta(destino, ev.step.label);
      const s = (destino.layout.sliders || [])[0];
      if (k < 0 || !s || k === vueltaActiva(destino)) return;
      sincronizando = true;
      Plotly.update(destino, s.steps[k].args[0], {"sliders[0].active": k})
        .then(function () {
          if (destino.__reaplicar) destino.__reaplicar();
          sincronizando = false;
        });
    });
  }

  enlazar(g2, g3);
  enlazar(g3, g2);
})();

// ---------------------------------------------------------------------------
// El PDF sale de la vuelta que marque el slider
// ---------------------------------------------------------------------------
// El estado del slider vive en el navegador. El go.Figure que guarda el
// servidor sigue como se construyo. Justo antes de que el enlace navegue se le
// añade &vuelta=<etiqueta del paso activo>, que es lo que el endpoint /pdf ya
// sabe atender. Sin esto, el boton bajaria siempre la primera vuelta
(function () {
  document.querySelectorAll('a.boton[data-grafica]').forEach(function (a) {
    const base = a.getAttribute("href");
    a.addEventListener("click", function () {
      const gd = document.getElementById(a.dataset.grafica);
      const s = gd && gd.layout && (gd.layout.sliders || [])[0];
      if (!s || !s.steps || !s.steps.length) return;   // figura sin slider
      const paso = s.steps[vueltaActiva(gd)];
      a.setAttribute("href", base + "&vuelta=" + encodeURIComponent(paso.label));
    });
  });
})();
