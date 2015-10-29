Informações úties:

    General Files:
        katana_300_6m180.urdf.xacro:
            Ficheiro com as informações todas do katana.
            
        _JointMovementGoal.py:
            Código python automaticamente gerado que explica as mensagens do katana.

    ROS Packages:
        joint_state_publisher:
            Cria publisher dos estados das joints de um robot que esteja definido nos parametros do Rospy.

        fake_katana_joint_publisher:
            Cria um publisher das joints do katana que publica /joint_states sempre as mesmas mensagens.
