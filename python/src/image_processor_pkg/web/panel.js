// camara -> coche -> [[x, y], ...] en pixeles de esa camara
const trazados = {};
const metas = {};      // camara -> [[x1, y1], [x2, y2]]
let coches = [];
let vista = null;      // {escala, dx, dy} del encuadre; null = recalcular
let totalPuntos = 0;
let arrastrando = null;
let saltoCorte = 60;   // Lo fija el servidor (4 x paso del trazado)

// Offsets del encaje manual entre camaras, se guardan en el navegador porque
// dependen de como esten colocadas las camaras en este montaje
const CLAVE = "offsets_camaras_tfg";
let offsets = JSON.parse(localStorage.getItem(CLAVE) || "{}");

// Ancho de la "plaza" de cada camara al aparecer, para que arranquen separadas
// Mismo criterio que ANCHO_PLAZA_CAMARA en analisis/figuras.py
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
    // Cada trazado llega entero, pero solo cuando ha cambiado
    for (const coche in cam.trazos) trazados[cam.id][coche] = cam.trazos[coche];
    if (cam.meta) metas[cam.id] = cam.meta;
    puntos += cam.total_puntos;
  }
  // Mientras el circuito se sigue dibujando el encuadre se rehace solo
  // Si la persona esta arrastrando no se toca, o el mapa se le mueve
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
    // rato es normal. Se marca en rojo a partir de 15 s, que ya son varias
    // vueltas sin aparecer
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
  // meta antes
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

    // En modo manual no hay PWM porque conduce una persona, la celda va vacia
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
  // Solo se sigue al ultimo evento si ya se estaba mirando el final
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
  // setTransform y no scale, que se iria acumulando en cada redimensionado
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ancho: r.width, alto: r.height };
}

// Todos los puntos de una camara, ya trasladados al plano comun con su offset
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
  // Escala unica para los dos ejes, para que el circuito no salga deformado
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

    // La linea de carrera de cada coche, en su color y apagada para que los
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
        // no ve, y unirlo pintaria una recta por donde no hay pista
        const salto = i > 0 && Math.hypot(x - pts[i-1][0], y - pts[i-1][1]) > saltoCorte;
        (i && !salto) ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
      });
      // Se cierra el anillo: la lista es lineal pero el circuito no, asi que
      // el ultimo punto y el primero quedan sin unir
      // Pasa por el mismo filtro que el resto, si estan a mas de saltoCorte no
      // se unen, que es el caso de la camara que solo ve un trozo del circuito
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

    // La trasera se pinta hueca y unida a la delantera: esa rayita es el derrape
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
  // Se elige la camara con el punto de trazado mas cercano al raton
  // El limite va en pixeles de pantalla, para que agarrar cueste lo mismo con
  // el mapa cerca o lejos
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
  // El raton se mueve en pixeles de pantalla y el offset en pixeles de camara,
  // asi que hay que dividir por la escala del encuadre
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
  Object.keys(trazados).sort().forEach(offsetDe);  // Vuelven a sus plazas
  guardarOffsets(); vista = null; dibujar();
};

document.getElementById("btn-copiar").onclick = () => {
  // Se normaliza para que la primera camara quede en (0, 0), que es el formato
  // que espera OFFSETS_CAMARAS en analisis/figuras.py
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
    () => alert(texto));   // Sin permiso de portapapeles se enseña para copiarlo
};

window.addEventListener("resize", () => { vista = null; dibujar(); });
dibujar();
