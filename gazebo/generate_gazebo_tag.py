import matplotlib.pyplot as plt

def generate_sdf(filename, square_size, num_squares):
    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="aruco_robot">
    <link name="aruco_link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.005 0.005 0.005</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>0.0</mass>
        <inertia>
          <ixx>0.0</ixx>
          <iyy>0.0</iyy>
          <izz>0.0</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.005 0.005 0.005</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
"""

    sdf_footer = """
  </model>
</sdf>
"""

    link_template = """
    <link name="arucobox_{i}_link">
      <pose>{x} {y} 0 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>{square_size} {square_size} 0.01</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
    <joint name="arucobox_{i}_joint" type="fixed">
      <parent>aruco_link</parent>
      <child>arucobox_{i}_link</child>
    </joint>
"""

    with open(filename, 'w') as f:
        f.write(sdf_header)
        
        index = 1
        half_size = (square_size * (num_squares - 1)) / 2  
        for i in range(num_squares):
            for j in range(num_squares):
                x = -half_size + i * square_size
                y = -half_size + j * square_size
                f.write(link_template.format(i=index, x=x, y=y, square_size=square_size))
                index += 1

        f.write(sdf_footer)

def generate_grid_image(num_squares, square_size, image_filename):
    fig, ax = plt.subplots()
    
    for i in range(num_squares + 1):
        ax.plot([0, num_squares], [i, i], color='black')
        ax.plot([i, i], [0, num_squares], color='black') 

    index = 1
    for i in range(num_squares):
        for j in range(num_squares):
            ax.text(j + 0.5, num_squares - i - 0.5, str(index),
                    va='center', ha='center')
            index += 1

    ax.set_xlim(0, num_squares)
    ax.set_ylim(0, num_squares)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal')
    ax.set_title(f'Quadrillage de {num_squares} carrés de cotés avec IDs')

    plt.savefig(image_filename)
    

generate_sdf('aruco2.urdf', square_size=0.03, num_squares=10)

generate_grid_image(num_squares=10, square_size=0.03, image_filename='aruco_grid.png')