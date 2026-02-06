# VR_usv

Este paquete se ha hecho modificando **articubot_one** desarrollado por **Jhon Newans** https://github.com/joshnewans/articubot_one


---

### Resumen de funcionalidades modificades/añadidas:

* Se ha modificado el **setup de las cámaras originales**
* Se ha añadido un **lunch para lo relacionado con vr** (procesado de imágenes y publicación en servidor web)
* Se ha creado un **script que desdistorsiona, sincroniza y pega** las imágenes de ambas cámaras

---

### Paquetes usados:

* **image_proc**
* **web_video_server**

---

### Librerias adicionales:

* **OpenCV**

* 
### Como lanzar una simulación mínima:

* Ejecutar **simular.sh**
* Lanzar en el puerto 80 el contenido con **sudo python3 -m http.server 80 --bind 0.0.0.0**
* Entrar desde ios o android y guardar url en inicio para visualizar en pantalla completa
