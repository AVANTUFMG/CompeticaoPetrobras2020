### Fork do MRS dos pacotes do Hector(necessários)

Caso não tenha os pacotes do hector, precisa fazer um git clone desse repositório na sua pasta /src, dentro do seu workspace, e então fazer um novo "catkin build" e "source /devel/setup.bash"
  - https://github.com/ctu-mrs/hector_slam



### Instruções de instalação gerais:
  - Siga o processo de instalação do https://github.com/ctu-mrs/mrs_uav_system ( Na aba de Installation );
  - No diretório do home/workspace/src;
  - Faça o git clone desse pacote;
  - Execute um catkin build  
  - Siga as instruções do video para configurar as fases:
  
    https://youtu.be/aM0vAr_YDT8
    
  - Pelo terminal acesse o diretório da pasta start, presente nesse pacote;
  - Execute o comando no terminal:
  
    $ ./start.sh

Apos isso inicie a simulação no gazebo apertando no play na parte inferior, a simulação deve iniciar, com o drone na origem.
- Para alterar a posição de spawn do drone acesse o arquivo uav1_pos.yaml na pasta start
- Na fase 4 é sugerido que para o levantamento das caixas utilizem o plugin gazebo_ros_link, instruções de como usar no git: https://github.com/pal-robotics/gazebo_ros_link_attacher
- Sensores não disponiveis:

  --enable-magnetic-gripper

  --enable-mobius-camera-back-left

  --enable-mobius-camera-back-right

  --enable-ouster

  --enable-pendulum

  --enable-realsense-top

  --enable-teraranger

  --enable-uv-camera

  --enable-uv-leds

  --enable-uv-leds-beacon

  --enable-whycon-box
