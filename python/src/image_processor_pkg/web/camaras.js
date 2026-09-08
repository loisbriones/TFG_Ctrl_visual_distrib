// ---------------------------------------------------------------------------
// Mosaico de camaras. Cada una es un <img src="/video/<camara>"> con un MJPEG,
// y de mostrar el video se encarga el navegador
// Aqui solo se pregunta cada 2 s que camaras hay (/estado_camaras), se crean sus
// recuadros y se reabre el video de las que se hayan quedado sin imagen
// ---------------------------------------------------------------------------
const mosaico = document.getElementById("mosaico");

// camara -> {caja, img, etiqueta, boton, pausada}
const tiles = {};

const PERIODO_SONDEO = 2000;

// Cada cuanto se reabre el video de una camara que no da imagen
// Tiene que ser >= el SIN_IMAGEN del nodo (10 s), o se acumulan conexiones
const PERIODO_REINTENTO = 10000;

function urlVideo(cam) {
  // El ?t=<ahora> no lo lee nadie, esta para que el navegador no reutilice de su
  // cache la conexion anterior, que ya esta cerrada
  return `/video/${cam}?t=${Date.now()}`;
}

function crearTile(cam) {
  const caja = document.createElement("div");
  caja.className = "camara-caja";

  const barra = document.createElement("div");
  barra.className = "camara-barra";

  const titulo = document.createElement("span");
  titulo.className = "camara-nombre";
  titulo.textContent = cam;

  const etiqueta = document.createElement("span");
  etiqueta.className = "camara-estado";

  const boton = document.createElement("button");
  boton.textContent = "pausa";

  const img = document.createElement("img");
  img.className = "camara-img";
  img.alt = cam;
  img.src = urlVideo(cam);

  const tile = { caja, img, etiqueta, boton, pausada: false, reintento: Date.now() };

  boton.onclick = () => {
    tile.pausada = !tile.pausada;
    if (tile.pausada) {
      // Quitar el src corta la conexion: el nodo ve que ya no queda nadie
      // mirando esta camara y se da de baja de su topic
      img.removeAttribute("src");
      boton.textContent = "ver";
      caja.classList.add("pausada");
    } else {
      img.src = urlVideo(cam);
      boton.textContent = "pausa";
      caja.classList.remove("pausada");
    }
    pintarEstado(cam, false);
  };

  barra.append(titulo, etiqueta, boton);
  caja.append(barra, img);
  mosaico.appendChild(caja);
  tiles[cam] = tile;
  return tile;
}

function pintarEstado(cam, hayImagen) {
  const tile = tiles[cam];
  if (tile.pausada) {
    tile.etiqueta.textContent = "en pausa";
    tile.etiqueta.className = "camara-estado";
  } else if (hayImagen) {
    tile.etiqueta.textContent = "en directo";
    tile.etiqueta.className = "camara-estado viva";
  } else {
    // O la camara tiene camera.debug en False, o esta arrancando, o se cayo
    tile.etiqueta.textContent = "sin imagen";
    tile.etiqueta.className = "camara-estado muerta";
  }
}

async function sondear() {
  let camaras;
  try {
    camaras = await (await fetch("/estado_camaras")).json();
  } catch (e) {
    document.getElementById("vivo").className = "desconectado";
    return;
  }
  document.getElementById("vivo").className = "punto-vivo";

  for (const c of camaras) {
    if (!tiles[c.id]) crearTile(c.id);
    const tile = tiles[c.id];

    // Sin imagen y sin estar pausada: se reabre el video
    if (!c.imagen && !tile.pausada
        && Date.now() - tile.reintento > PERIODO_REINTENTO) {
      tile.reintento = Date.now();
      tile.img.src = urlVideo(c.id);
    }
    if (c.imagen) tile.reintento = Date.now();
    pintarEstado(c.id, c.imagen);
  }

  mosaico.classList.toggle("vacio", camaras.length === 0);
}

sondear();
setInterval(sondear, PERIODO_SONDEO);
