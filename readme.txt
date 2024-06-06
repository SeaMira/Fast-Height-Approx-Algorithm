En caso de utilizar está estructura, recordar mencionar sus archivos cpp en CPP_LIST y los archivos hpp en HPP_LIST. No es necesario llamar a cmake .. por cada modificación del código, en caso de pequeños cambios solo llamar a make y actualizara CMakeCache con los cambios hechos.

Para construir el proyecto abrir en terminal carpeta build/ y ejecutar el comando:

cmake ..

Para generar el ejecutable usar el comando:

make

Para ejecutar:

./playCircle

Para ejecutar tests:

ctest . 

o el comando:

./test/CircleTests

Para ejectuar test individuales:

ctest -R <nombre test_suite_name, primer nombres despues de "TEST(">

Links de interes:

Estructura proyectos cmake: https://cliutils.gitlab.io/modern-cmake/chapters/basics/structure.html
assertions gtest: https://google.github.io/googletest/reference/assertions.html
Test Fixtures (override gtest): https://google.github.io/googletest/primer.html
CGAL con cmake: https://doc.cgal.org/latest/Manual/devman_create_and_use_a_cmakelist.html