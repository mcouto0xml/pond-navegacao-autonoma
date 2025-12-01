# pond-navegacao-autonoma

Para rodar é necessário colocar somente as pastas `my_part1`e `my_part2` dentro do retório `src`. 
Considerando claro, que o repositório `culling_games`está clonado, o usuário está dentro de um ambiente de ros2.
O que falta agora é só dar `colcon build`, `source install/setup.bash` para buildar o ambiente e testar:
Parte 1: `ros2 run my_part1 get_map_client`
Parte 2: `ros2 run my_part2 maze_mapper`


Vídeo explicativo da ponderada abaixo:
https://youtu.be/IPxKLhrUPYc