// ---------------------------------------------------------------------------
// Estado del navegador. El servidor manda solo lo que cambia, asi que la
// pagina va acumulando: el trazado de cada camara crece punto a punto y los
// eventos se van apilando. Cuando el servidor avisa con "reinicio" (primera
// conexion, recarga, o EventSource reconectando tras un corte) se tira todo y
// se vuelve a construir.
// ---------------------------------------------------------------------------
// camara -> coche -> [[x, y], ...] en pixeles DE ESA CAMARA. Un trazado por
// coche porque cada uno va por su carril: son dos curvas paralelas, y juntas
// en una sola lista el circuito salia dibujado en zigzag entre los dos carriles
const trazados = {};
const metas = {};      // camara -> [[x1, y1], [x2, y2]]
let coches = [];
let vista = null;      // {escala, dx, dy} del encuadre; null = recalcular
let totalPuntos = 0;
let arrastrando = null;
let saltoCorte = 60;   // lo fija el servidor (4 x paso del trazado)

// Offsets del encaje manual entre camaras. Se guardan en el navegador porque
// dependen de COMO estan puestas las camaras en este montaje concreto: una vez
// encajado, no hay que volver a tocarlo aunque se recargue la pagina.
const CLAVE = "offsets_camaras_tfg";
let offsets = JSON.parse(localStorage.getItem(CLAVE) || "{}");

// Ancho de la "plaza" de cada camara al aparecer: 640 px de imagen + margen.
// Mismo criterio que ANCHO_PLAZA_CAMARA en analisis/figuras.py, para que las
// camaras arranquen separadas y no una encima de otra
const ANCHO_PLAZA = 660;

const lienzo = document.getElementById("mapa");
const ctx = lienzo.getContext("2d");

function offsetDe(cam) {
  if (!offsets[cam]) {
    // Cada camara nueva entra en la siguiente plaza libre, por orden
    const n = Object.keys(offsets).length;
    offsets[cam] = [n * ANCHO_PLAZA, 0];
  }
  return offsets[cam];
}

function guardarOffsets() { localStorage.setItem(CLAVE, JSON.stringify(offsets)); }

// ---------------------------------------------------------------------------
// Recepcion
// ---------------------------------------------------------------------------
const fuente = new EventSource("/stream");
fuente.onmessage = (e) => {
  document.getElementById("vivo").className = "punto-vivo";
  const d = JSON.parse(e.data);

  if (d.reinicio) {
    for (const k in trazados) delete trazados[k];
    document.getElementById("eventos").innerHTML = "";
    vista = null; totalPuntos = 0;
  }

  document.getElementById("reloj").textContent = d.reloj;
  coches = d.coches;

  saltoCorte = d.salto_corte;
  let puntos = 0;
  for (const cam of d.camaras) {
    if (!trazados[cam.id]) { trazados[cam.id] = {}; offsetDe(cam.id); vista = null; }
    // Cada trazado llega entero, pero solo cuando ha cambiado (los puntos se
    // intercalan en el servidor, asi que no se pueden ir añadiendo aquí)
    for (const coche in cam.trazos) trazados[cam.id][coche] = cam.trazos[coche];
    if (cam.meta) metas[cam.id] = cam.meta;
    puntos += cam.total_puntos;
  }
  // Mientras el circuito se sigue dibujando (la vuelta de calibracion) el
  // encuadre se rehace solo para que no se salga nada; una vez cerrado el
  // trazado deja de cambiar. Si la persona esta arrastrando no se toca, que
  // si no el mapa se le movería debajo del raton
  if (puntos !== totalPuntos && !arrastrando) { totalPuntos = puntos; vista = null; }

  pintarSalud(d);
  pintarTabla(d);
  pintarEventos(d.eventos);
  dibujar();
};
fuente.onerror = () => { document.getElementById("vivo").className = "desconectado"; };

// ---------------------------------------------------------------------------
// Barra de salud y aviso de calibracion
// ---------------------------------------------------------------------------
function pintarSalud(d) {
  const trozos = [];
  for (const cam of d.camaras) {
    // Una camara solo publica mientras ve al coche, asi que estar callada un
    // rato es NORMAL (el coche esta en el trozo de otra camara). Solo se marca
    // en rojo si lleva mas de 15 s sin dar señales, que a 4-6 s por vuelta son
    // ya tres vueltas enteras sin aparecer: eso si es una Raspberry caida
    const muerta = cam.desde === null || cam.desde > 15;
    const cuando = cam.desde === null ? "sin datos" :
                   cam.desde < 1 ? "ahora" : `hace ${cam.desde.toFixed(0)} s`;
    trozos.push(`<div class="chip${muerta ? " mala" : ""}">` +
                `<b>${cam.id}</b> ${cam.hz.toFixed(1)} Hz · ${cuando}</div>`);
  }
  for (const c of d.coches) {
    const lat = c.pipeline_ms === null ? "—" : c.pipeline_ms.toFixed(1) + " ms";
    trozos.push(`<div class="chip"><b>${c.nombre}</b> pipeline ${lat}</div>`);
  }
  if (!trozos.length) trozos.push('<div class="chip">esperando a que publique alguien…</div>');
  document.getElementById("salud").innerHTML = trozos.join("");

  const calibrando = d.coches.filter(c => c.calibrando).map(c => c.nombre);
  const aviso = document.getElementById("aviso");
  aviso.hidden = calibrando.length === 0;
  aviso.textContent = "📐 CALIBRANDO — " + calibrando.join(", ") +
                      ": aprendiendo la trayectoria, la carrera aún no ha empezado";
}

// ---------------------------------------------------------------------------
// Tabla de clasificacion
// ---------------------------------------------------------------------------
const seg = (t) => t === null ? "—" : t.toFixed(3);

function pintarTabla(d) {
  // Orden de carrera: manda el numero de vueltas y, a igualdad, quien cruzo
  // meta antes. No hay posicion "en pista" porque nadie la calcula: cada
  // camara ve pixeles suyos y no hay una distancia comun al circuito
  const orden = [...d.coches].sort((a, b) =>
    (b.vuelta - a.vuelta) || ((a.t_cruce ?? Infinity) - (b.t_cruce ?? Infinity)));

  document.getElementById("tabla").innerHTML = orden.map((c, i) => {
    let claseUlt = "";
    if (c.ultimo_tiempo !== null) {
      if (c.es_mejor_sesion && c.ultimo_tiempo === c.mejor_tiempo) claseUlt = "morado";
      else if (c.ultimo_tiempo === c.mejor_tiempo) claseUlt = "verde";
      else claseUlt = "amarillo";
    }
    const delta = c.delta === null ? "—" :
      `<span class="${c.delta < 0 ? "verde" : "amarillo"}">${c.delta > 0 ? "+" : ""}${c.delta.toFixed(3)}</span>`;

    let estado = '<span class="estado ok">OK</span>';
    if (c.sin_senal) estado = '<span class="estado gris">SIN SEÑAL</span>';
    else if (c.derrapando) estado = `<span class="estado derrapa">DERRAPA ${c.dist_derrape}px</span>`;

    // En modo manual no hay PWM: el controlador no publica pwd porque conduce
    // una persona, y la celda se queda vacia a proposito
    let pwm = '<span class="apagado">—</span>';
    if (c.pwm !== null) {
      const pc = Math.max(0, Math.min(100, (c.pwm - d.v_min) / (d.v_max - d.v_min) * 100));
      pwm = `<div class="barra"><i style="width:${pc}%"></i><span>${c.pwm}</span></div>`;
    }

    return `<tr class="${c.sin_senal ? "perdido" : ""}">
      <td class="pos">${i + 1}</td>
      <td class="coche"><span class="tira" style="background:${c.color}"></span>${c.nombre}</td>
      <td class="apagado">${c.carril ?? "—"}</td>
      <td>${c.vuelta}</td>
      <td class="${claseUlt}">${seg(c.ultimo_tiempo)}</td>
      <td>${delta}</td>
      <td>${seg(c.mejor_tiempo)}${c.mejor_vuelta ? ` <span class="apagado">v${c.mejor_vuelta}</span>` : ""}</td>
      <td>${pwm}</td>
      <td>${estado}</td>
    </tr>`;
  }).join("") || '<tr><td colspan="9" class="apagado">Sin coches todavía.</td></tr>';
}

// ---------------------------------------------------------------------------
// Eventos
// ---------------------------------------------------------------------------
function pintarEventos(eventos) {
  if (!eventos.length) return;
  const caja = document.getElementById("eventos");
  const abajo = caja.scrollTop + caja.clientHeight >= caja.scrollHeight - 20;
  for (const ev of eventos) {
    const fila = document.createElement("div");
    fila.className = "ev " + ev.tipo;
    fila.innerHTML = `<time>${ev.reloj}</time><span></span>`;
    fila.lastChild.textContent = ev.texto;
    caja.appendChild(fila);
  }
  while (caja.childElementCount > 200) caja.removeChild(caja.firstChild);
  // Solo se sigue al ultimo evento si ya se estaba mirando el final: si
  // alguien ha subido a leer algo, no se le arrastra hacia abajo
  if (abajo) caja.scrollTop = caja.scrollHeight;
}

// ---------------------------------------------------------------------------
// Mapa
// ---------------------------------------------------------------------------
function ajustarLienzo() {
  const r = lienzo.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  lienzo.width = Math.round(r.width * dpr);
  lienzo.height = Math.round(r.height * dpr);
  // La transformacion se FIJA (setTransform, no scale) en cada ajuste: con
  // scale se iría acumulando en cada redimensionado de la ventana
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ancho: r.width, alto: r.height };
}

// Todos los puntos de una cámara (los de todos sus coches), ya trasladados al
// plano común con el offset de esa cámara
function* puntosDe(cam) {
  const [ox, oy] = offsetDe(cam);
  for (const coche in trazados[cam])
    for (const [x, y] of trazados[cam][coche]) yield [x + ox, y + oy];
}

function limites() {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const cam in trazados)
    for (const [x, y] of puntosDe(cam)) {
      x0 = Math.min(x0, x); x1 = Math.max(x1, x);
      y0 = Math.min(y0, y); y1 = Math.max(y1, y);
    }
  return x0 === Infinity ? null : { x0, y0, x1, y1 };
}

function encuadrar(ancho, alto) {
  const l = limites();
  if (!l) return null;
  const m = 24;
  // Escala UNICA para los dos ejes: el circuito tiene que salir sin deformar,
  // igual que el scaleanchor 1:1 de las figuras del analisis
  const escala = Math.min((ancho - 2 * m) / Math.max(l.x1 - l.x0, 1),
                          (alto - 2 * m) / Math.max(l.y1 - l.y0, 1));
  return {
    escala,
    dx: m - l.x0 * escala + (ancho - 2 * m - (l.x1 - l.x0) * escala) / 2,
    dy: m - l.y0 * escala + (alto - 2 * m - (l.y1 - l.y0) * escala) / 2,
  };
}

const aPantalla = (x, y) => [x * vista.escala + vista.dx, y * vista.escala + vista.dy];

function dibujar() {
  const { ancho, alto } = ajustarLienzo();
  ctx.clearRect(0, 0, ancho, alto);

  if (!vista) vista = encuadrar(ancho, alto);
  if (!vista) {
    ctx.fillStyle = "#6b7079"; ctx.font = "13px system-ui";
    ctx.fillText("El circuito se dibuja solo con las posiciones del coche.", 16, 28);
    ctx.fillText("Da una vuelta (o reproduce un bag) y aparecerá aquí.", 16, 48);
    return;
  }

  const colorDe = {};
  for (const c of coches) colorDe[c.nombre] = c.color;

  for (const cam in trazados) {
    const [ox, oy] = offsetDe(cam);
    let primero = null;

    // La línea de carrera de cada coche, en su color y apagada para que los
    // puntos de los coches destaquen encima
    for (const coche in trazados[cam]) {
      const pts = trazados[cam][coche];
      if (!pts.length) continue;
      primero = primero || pts[0];

      ctx.strokeStyle = colorDe[coche] || "#5a6270";
      ctx.globalAlpha = 0.55;
      ctx.lineWidth = 2.5;
      ctx.lineJoin = ctx.lineCap = "round";
      ctx.beginPath();
      pts.forEach(([x, y], i) => {
        const [px, py] = aPantalla(x + ox, y + oy);
        // Se levanta el lapiz en los saltos: ese trozo es lo que esta camara
        // NO ve (el coche se sale del encuadre y vuelve a entrar por otro
        // lado), y unirlo pintaria una recta por donde no hay pista
        const salto = i > 0 && Math.hypot(x - pts[i-1][0], y - pts[i-1][1]) > saltoCorte;
        (i && !salto) ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      });
      // Y se CIERRA EL ANILLO. La lista es lineal pero el circuito no: cuando
      // el coche completa la vuelta, el ultimo punto queda pegado al primero,
      // solo que nadie dibujaba el segmento que los une. Quedaba un hueco en
      // mitad del trazado, en un sitio distinto cada vez (donde estuviera el
      // coche al conectarse el panel) y sin ninguna oclusion que lo explicara.
      //
      // El cierre pasa por el MISMO filtro que el resto de segmentos: si los
      // dos extremos estan a mas de saltoCorte no se unen. Asi se respetan los
      // dos casos en los que ese segmento no existe: una camara que solo ve un
      // trozo del circuito (cadena abierta), y las sesiones en las que el
      // trazado arranca con el coche entrando en el encuadre, donde el primer
      // punto no esta sobre el trazado cerrado sino a un lado.
      const [ax, ay] = pts[0], [zx, zy] = pts[pts.length - 1];
      if (pts.length > 2 && Math.hypot(zx - ax, zy - ay) <= saltoCorte)
        ctx.lineTo(...aPantalla(ax + ox, ay + oy));
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
    if (!primero) continue;

    if (metas[cam]) {
      ctx.strokeStyle = "#ff9f43";
      ctx.lineWidth = 3;
      ctx.beginPath();
      const [a, b] = metas[cam];
      ctx.moveTo(...aPantalla(a[0] + ox, a[1] + oy));
      ctx.lineTo(...aPantalla(b[0] + ox, b[1] + oy));
      ctx.stroke();
    }

    // Etiqueta de la camara, para saber que bloque se esta arrastrando
    const [ex, ey] = aPantalla(primero[0] + ox, primero[1] + oy);
    ctx.fillStyle = "#6b7079"; ctx.font = "11px system-ui";
    ctx.fillText(cam, ex + 8, ey - 8);
  }

  for (const c of coches) {
    if (!c.front || !c.camara || !trazados[c.camara]) continue;
    const [ox, oy] = offsetDe(c.camara);
    const f = aPantalla(c.front[0] + ox, c.front[1] + oy);

    // La trasera se pinta hueca y unida a la delantera: esa rayita ES el
    // derrape (el algoritmo mide justo cuanto se separa de la trayectoria)
    if (c.back) {
      const b = aPantalla(c.back[0] + ox, c.back[1] + oy);
      ctx.strokeStyle = c.derrapando ? "#ff6b6b" : c.color;
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(...f); ctx.lineTo(...b); ctx.stroke();
      ctx.beginPath(); ctx.arc(b[0], b[1], 3.5, 0, 6.284); ctx.stroke();
    }
    ctx.fillStyle = c.sin_senal ? "#6b7079" : c.color;
    ctx.beginPath(); ctx.arc(f[0], f[1], 6, 0, 6.284); ctx.fill();
    if (c.derrapando) {
      ctx.strokeStyle = "#ff6b6b"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(f[0], f[1], 10, 0, 6.284); ctx.stroke();
    }
  }
}

// --- Encaje manual: arrastrar cada camara hasta que el circuito sea continuo
function camaraEn(px, py) {
  // Se elige la camara con el punto de trazado mas cercano al raton; el limite
  // en pixeles de PANTALLA (no de mundo) para que agarrar cueste lo mismo
  // este el mapa muy alejado o muy cerca
  let mejor = null, mejorD = 30;
  for (const cam in trazados)
    for (const [x, y] of puntosDe(cam)) {
      const [sx, sy] = aPantalla(x, y);
      const d = Math.hypot(sx - px, sy - py);
      if (d < mejorD) { mejorD = d; mejor = cam; }
    }
  return mejor;
}

lienzo.addEventListener("mousedown", (e) => {
  if (!vista) return;
  const r = lienzo.getBoundingClientRect();
  const cam = camaraEn(e.clientX - r.left, e.clientY - r.top);
  if (!cam) return;
  arrastrando = { cam, x: e.clientX, y: e.clientY };
  lienzo.classList.add("arrastrando");
});

window.addEventListener("mousemove", (e) => {
  if (!arrastrando) return;
  // El raton se mueve en pixeles de pantalla y el offset esta en pixeles de
  // camara: hay que dividir por la escala del encuadre
  const o = offsetDe(arrastrando.cam);
  o[0] += (e.clientX - arrastrando.x) / vista.escala;
  o[1] += (e.clientY - arrastrando.y) / vista.escala;
  arrastrando.x = e.clientX; arrastrando.y = e.clientY;
  dibujar();
});

window.addEventListener("mouseup", () => {
  if (!arrastrando) return;
  arrastrando = null;
  lienzo.classList.remove("arrastrando");
  guardarOffsets();
});

document.getElementById("btn-encajar").onclick = () => { vista = null; dibujar(); };

document.getElementById("btn-reset").onclick = () => {
  offsets = {};
  Object.keys(trazados).sort().forEach(offsetDe);  // vuelven a sus plazas
  guardarOffsets(); vista = null; dibujar();
};

document.getElementById("btn-copiar").onclick = () => {
  // Se normaliza para que la primera camara quede en (0, 0): asi el resultado
  // es exactamente lo que espera OFFSETS_CAMARAS en analisis/figuras.py, y el
  // analisis posterior del bag dibuja el circuito con el mismo encaje que se
  // acaba de ajustar aqui a ojo
  const cams = Object.keys(trazados).sort();
  if (!cams.length) return;
  const [bx, by] = offsetDe(cams[0]);
  const cuerpo = cams.map(c => {
    const [x, y] = offsetDe(c);
    return `    "${c}": (${(x - bx).toFixed(1)}, ${(y - by).toFixed(1)}),`;
  }).join("\n");
  const texto = "OFFSETS_CAMARAS = {\n" + cuerpo + "\n}";
  navigator.clipboard.writeText(texto).then(
    () => { const b = document.getElementById("btn-copiar");
            b.textContent = "✓ copiado"; setTimeout(() => b.textContent = "copiar offsets", 1500); },
    () => alert(texto));   // sin permiso de portapapeles (http a secas): se enseña
};

window.addEventListener("resize", () => { vista = null; dibujar(); });
dibujar();
