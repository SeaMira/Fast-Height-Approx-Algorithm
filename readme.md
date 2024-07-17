# Aproximación de Terrenos y Campos de Altura

En este proyecto del ramo de Geometría Computacional CC5502-1 - Otoño 2024 se tuvo el principal objetivo de implementar un algoritmo capaz de recrear un campo de alturas/terreno/malla con su función de altura H(x,y) (esto retorna un valor para z en esa coordenada) en un mesh triangular aproximado que minimize el error entre los datos utilizados y los designados por la data.

Principalmente puede ser utilizado para renderizado de terreno y un mejor o peor nivel de detalle.

El algoritmo y las ideas utilizadas fueron extraídas del paper "Fast Polygonal Approximation of
Terrains and Height Fields", adjunto en los archivos. En este documento se presenta la base de la idea a desarrollar, mencionando la forma de generación del terreno, estructuras de datos, eficiencia e incluso seudocódigo del método que generará la malla triangular que represente el terreno.

Herramientas utilizadas:

- Lenguaje de programación C++
- Biblioteca de algoritmos geométricos CGAL
- Librería de procesamiento de imágenes ImageMagick

## Delaunay Greedy Insertion

Se le llama a un método de refinamiento de la malla "Greedy Insertion" pues en cada iteración inserta el punto con mayor error, y una vez insertado el punto no se quitará. Es un método secuencial que inserta puntos en una triangulación de Delaunay.

La malla triangular producida es de Delaunay (solo en 2D, sin contar la altura) porque así se tienen triángulos con ángulo mínimo más grande, minimizando la cantidad de "triángulos estirados" que aportarían de peor forma a la estética de la malla.

La implementación tiene de base el siguiente seudocódigo:

![plot](./readmeImages\seudoCode1.png)
![plot](./readmeImages\seudoCode2.png)

Se explicarán las estructuras, funciones y definiciones usadas del mismo a continuación.

### Error

Se decidió utilizar el mismo tipo de error mencionado en el código, el cuál describe el error máximo localmente por triángulo presente en la triangulación. El error es la diferencia absoluta entre el valor de z interpolado que tiene un par ordenado (x,y) dentro del plano que describe el triángulo y el valor H(x,y) dado por el campo de altura.

Este valor de error describe un candidato de punto a insertar en este triángulo, insertándose en el heap (error es la llave y punto el contenido).

### Heap

Un heap binario es una estructura de datos en forma de árbol binario que cumple con la propiedad del heap. Existen dos tipos principales de heaps binarios: el heap máximo y el heap mínimo. A continuación, se describen sus propiedades y cómo funcionan:

En un heap máximo, para cada nodo N, la clave de N es mayor o igual que las claves de sus hijos. Esto asegura que el nodo con la clave máxima esté en la raíz del árbol.

#### HeapNode

Es una clase genérica implementada para el proyecto que representa un nodo en un heap binario. Cada 'HeapNode' contiene una clave (key) y un contenido (content). La clave se utiliza para mantener la propiedad del heap (en este caso, un heap máximo), mientras que el contenido puede ser cualquier tipo de dato asociado con esa clave.

Partes Importantes de HeapNode

- Miembros Públicos:

  - content: El contenido del nodo, que puede ser de cualquier tipo (CT).
  - key(): Un método que retorna la clave del nodo (KT).

- Miembros Privados:

  - \_key: La clave del nodo, utilizada para mantener la propiedad del heap.
  - heap_position: La posición del nodo en el vector del heap. Este valor se actualiza a medida que el nodo se mueve dentro del heap.

* Constructor Privado:

  - HeapNode(KT key, CT content): Constructor que inicializa el nodo con una clave y un contenido. Este constructor es privado y solo puede ser utilizado por la clase BMHeap, lo que garantiza que los nodos solo se crean dentro del contexto del heap.

#### BMHeap

El BMHeap es una clase que implementa un heap binario máximo utilizando nodos de tipo HeapNode. Este heap organiza los nodos según sus claves, manteniendo el nodo con la clave más alta en la raíz del heap.

Partes Importantes de BMHeap

- Miembros Públicos:

  - empty(): Retorna true si el heap está vacío.
  - clear(): Limpia todos los nodos del heap.
  - top(): Retorna una referencia al nodo con la clave más alta (la raíz de heap).
  - pop(): Elimina el nodo con la clave más alta del heap.

  - remove(HeapNode<KT, CT>& node): Elimina un nodo específico del heap.
  - update(HeapNode<KT, CT>& node, KT key): Actualiza la clave de un nodo y reordena el heap.
  - push(KT key, CT content): Inserta un nuevo nodo en el heap con una clave y contenido específicos.
  - index_of(const HeapNode<KT, CT>& node): Encuentra el índice permanente de un nodo en el vector de nodos.

- Miembros Privados:

  - permanent_nodes: Vector que almacena los nodos permanentemente. Los nodos nunca se eliminan de este vector.
  - heap: Vector que organiza los índices de los nodos en forma de heap.
  - key_map: Mapa que rastrea los nodos por su clave para facilitar el acceso.

- Métodos Privados:

  - reheap_up(HeapNode<KT, CT>& node): Reorganiza el heap moviendo un nodo hacia arriba hasta que se restaure la propiedad del heap.
  - reheap_down(HeapNode<KT, CT>& node): Reorganiza el heap moviendo un nodo hacia abajo hasta que se restaure la propiedad del heap.
  - reheap(HeapNode<KT, CT>& node): Determina si un nodo debe moverse hacia arriba o hacia abajo y llama a la función apropiada.
  - swap_nodes(HeapNode<KT, CT>& node1, HeapNode<KT, CT>& node2): Intercambia dos nodos en el heap y actualiza sus posiciones.

Entonces en este caso se utilizó un heap para poder almacenar en la raiz el el punto (y la cara con este punto) que genera el mayor error local de la triangulación.

### Mesh y Triángulos

Para implementar esta parte se utilizaron las librerías de Triangulations 2D y de Mesh surface de CGAL, pues en la primera se podría tener el "Mesh" (una triangulación) de Delaunay con la implementación que tiene CGAL de la misma: con cada inserción de puntos se adapta para que siga siendo de Delaunay, y se ocupa Mesh para luego poder llevarlo a un fromato ".obj".

### Triangle Plane

Dados los 3 vértices de un triángulo se calculan los coeficientes A, B, C y D que representan el plano que pasa por esos vértices tal que Ax+By+Cz+D=0, luego para obtener la altura interpolada basa con despejar z y reemplazar con los valores de x e y.

## Algoritmo

1. Inicialización de la Malla:
   Se comienza con una malla inicial muy simple que consiste en dos triángulos que cubren el dominio del campo de altura

2. Escaneo de Triángulos:
   Cada triángulo en la malla se escanea para encontrar el punto dentro del triángulo que tiene el mayor error de aproximación respecto al campo de altura real. Este error se calcula como la diferencia entre el valor de altura interpolado del triángulo y el valor real del campo de altura en ese punto.

- Para cada triángulo T:

  - Se encuentra el plano que define T.

  - Se inicializan variables para rastrear el mejor punto de inserción y el error máximo.

  - Para cada punto p seleccionado dentro de T:

    - Se calcula el error de p.

    - Si el error es mayor que el error máximo registrado, se actualizan el error máximo y el mejor punto de inserción.

3. Cambio en el Heap:

   - Se utiliza un heap para almacenar los errores máximos de los triángulos y para permitir una selección rápida del triángulo con el mayor error.
   - El heap se actualiza con los errores máximos y los puntos y triángulos correspondientes.

4. Inserción en la Malla:
   El punto con el mayor error se inserta en la malla.
   La malla se actualiza utilizando triangulación incremental de Delaunay, lo que garantiza que la malla resultante mantenga ciertas propiedades geométricas óptimas.
   Después de la inserción, se escanean nuevamente los triángulos afectados para recalcular sus errores y actualizar el heap.

5. Condición de Terminación:
   El proceso de inserción y actualización continúa hasta que se cumpla una condición de terminación predefinida, como alcanzar un número máximo de puntos o reducir el error por debajo de un umbral específico. En este caso se utilizó el primero.

## Complicaciones

Entre los problemas encontrados se encuentran los siguientes detalles:

- Selección de herramientas a utilizar: en un inicio se quizo hacer con los Mesh de CGAL e implementar las condiciones de Delaunay a mano, sin embargo, podría ser desgastador y engorroso. Se decidió cambiar a mitad del proceso por el uso de las triangulaciones como malla directamente.

- Errores de implementación de estructuras de datos: el heap implementado tuvo que ser corregido múltiples veces para poder almacenar los punteros a las posiciones correspondientes en el árbol, razón por la que maneja más de un vector de datos.

- Método de búsqueda de puntos dentro del triángulo: al momento de escanear los triángulos de la malla en búsqueda del punto de mayor error se complicó el decidir qué puntos estudiar y cuántos puntos, por lo que se decidió realizar el estudio desde el centroide del triángulo: desde aquí se estudiarían "steps" (parámetro por explicar) puntos en dirección a cada uno de los vértices, y en dirección a los puntos medios de los lados, eso hace un estudio de steps\*6 puntos por triángulo.

## Método de compilación y ejecución

Antes de proceder con los comandos se menciona que hay 2 maneras de probar la aplicación:

- Con una función de altura generada por la rasterización de un campo de altura (los campos de altura a los que se hace referencia son imágenes que representan alturas interpolando colores: image.png, image2.png y paper_image.png).

- Con una función de altura creada aleatoriamente con una función de "perlin noise".

En primer lugar entonces abrir una terminal en la carpeta principal del proyecto (nombre Fast-Height-Approx-Algorithm).

Para crear una build y compilar se deben ejecutar los siguientes comandos:

```bash
cmake -S . -B ./build
cmake --build ./build -j 10
```

Luego para ejecutar se debe dar uno de los siguientes comandos:

- Se genera con una función aleatoria:

```bash
./build/fham_simple_demo.exe 1 <points> <steps> <hlf grid side> <obj filename>
```

- Se genera con la función de altura del rasterizado de una imagen:

```bash
 ./build/fham_simple_demo.exe 2 <points> <steps> <max z> <in image> <obj filename>
```

Tal que:

- points: número de vértices a insertar en la malla.
- steps: cantidad de puntos a checkear en cada dirección al estudiar candidatos dentro del triángulo (más detalle en la sección "complicaciones" del readme)
- hlf grid side: mitad del tamaño del grid cuadrado que genera el primer comando.
- obj filename: nombre del archivo .obj de salida con la malla triangular.
- max z: máximo valor de retorno que puede entregar la función de altura generada de la rasterización.
- in image: nombre de la imagen a la cuál se le extraerá una función de altura en base a su rasterización.

Ejemplo de uso:

- Función aleatoria:

```bash
./build/fham_simple_demo.exe 1 5000 10 30.0 noiseout.obj
```

- Función de raster:

```bash
./build/fham_simple_demo.exe 2 5000 10 60.0 image.png rasterout.obj
```

## Finalización del proyecto

Con eso finalizaría este proyecto, gracias por la atención.

### Autores:

- Gonzalo Álvarez
- Sebastián Mira
