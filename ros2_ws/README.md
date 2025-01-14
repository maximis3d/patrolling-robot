
```
ros2_ws
├─ .gitignore
├─ dataset
│  ├─ data.yaml
│  ├─ labels
│  │  └─ img_0000.txt
│  ├─ labels.cache
│  ├─ runs
│  │  └─ detect
│  │     ├─ train
│  │     │  ├─ args.yaml
│  │     │  └─ weights
│  │     └─ train2
│  │        ├─ F1_curve.png
│  │        ├─ PR_curve.png
│  │        ├─ P_curve.png
│  │        ├─ R_curve.png
│  │        ├─ args.yaml
│  │        ├─ confusion_matrix.png
│  │        ├─ confusion_matrix_normalized.png
│  │        ├─ labels.jpg
│  │        ├─ labels_correlogram.jpg
│  │        ├─ results.csv
│  │        ├─ results.png
│  │        ├─ train_batch0.jpg
│  │        ├─ train_batch1.jpg
│  │        ├─ train_batch2.jpg
│  │        ├─ train_batch40.jpg
│  │        ├─ train_batch41.jpg
│  │        ├─ train_batch42.jpg
│  │        ├─ val_batch0_labels.jpg
│  │        ├─ val_batch0_pred.jpg
│  │        └─ weights
│  │           ├─ best.pt
│  │           └─ last.pt
│  ├─ yolo11n.pt
│  └─ yolov8n.pt
├─ src
│  ├─ patrolling_robot_master
│  │  ├─ launch
│  │  │  └─ master_launch.py
│  │  ├─ package.xml
│  │  ├─ patrolling_robot_master
│  │  │  └─ __init__.py
│  │  ├─ resource
│  │  │  └─ patrolling_robot_master
│  │  ├─ setup.cfg
│  │  ├─ setup.py
│  │  └─ test
│  │     ├─ test_copyright.py
│  │     ├─ test_flake8.py
│  │     └─ test_pep257.py
│  ├─ patrolling_robot_simulation
│  │  ├─ maps
│  │  │  ├─ map.pgm
│  │  │  └─ map.yaml
│  │  ├─ models
│  │  │  ├─ Floor_room
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  ├─ Floor.material
│  │  │  │  │  │  └─ Floor_room.material
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ carpet_1.jpg
│  │  │  │  │     ├─ carpet_2.jpg
│  │  │  │  │     ├─ carpet_3.jpg
│  │  │  │  │     ├─ carpet_4.jpg
│  │  │  │  │     ├─ cl1.jpg
│  │  │  │  │     ├─ cl2.jpg
│  │  │  │  │     ├─ cl3.jpg
│  │  │  │  │     ├─ cl4.jpg
│  │  │  │  │     ├─ cl5.jpg
│  │  │  │  │     ├─ cl6.jpg
│  │  │  │  │     ├─ cp1.jpg
│  │  │  │  │     ├─ cp2.jpg
│  │  │  │  │     ├─ cp3.jpg
│  │  │  │  │     ├─ cp4.jpg
│  │  │  │  │     ├─ cp5.png
│  │  │  │  │     ├─ cp6.jpg
│  │  │  │  │     ├─ cp7.jpg
│  │  │  │  │     ├─ cp8.jpg
│  │  │  │  │     ├─ hd1.jpg
│  │  │  │  │     ├─ hd2.gif
│  │  │  │  │     ├─ hd3.jpg
│  │  │  │  │     ├─ hd4.jpg
│  │  │  │  │     ├─ hd5.jpg
│  │  │  │  │     ├─ hd6.png
│  │  │  │  │     ├─ hd7.jpg
│  │  │  │  │     └─ hd8.jpg
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ Wall_Cabinet_36x15W
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_AirconditionerA_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_AirconditionerA_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_AirconditionerA_01_collision.DAE
│  │  │  │  │  ├─ aws_AirconditionerA_01_visual.DAE
│  │  │  │  │  └─ aws__Air conditioner_01.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_AirconditionerB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_AirconditionerB_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_AirconditionerB_01_collision.DAE
│  │  │  │  │  └─ aws_AirconditionerB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_BalconyTable_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_BalconyTable_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_BalconyTable_01_collision.DAE
│  │  │  │  │  └─ aws_BalconyTable_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Ball_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Ball_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Ball_01_collision.DAE
│  │  │  │  │  └─ aws_Ball_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Bed_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_BedA_01.png
│  │  │  │  │     ├─ aws_BedB_01.png
│  │  │  │  │     ├─ aws_BedC_01.png
│  │  │  │  │     └─ aws_Bed_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Bed_01_collision.DAE
│  │  │  │  │  └─ aws_Bed_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Board_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Board_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Board_01_collision.DAE
│  │  │  │  │  └─ aws_Board_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Carpet_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Carpet_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Carpet_01_collision.DAE
│  │  │  │  │  └─ aws_Carpet_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_ChairA_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_ChairA_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_ChairA_01_collision.DAE
│  │  │  │  │  └─ aws_ChairA_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_ChairD_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_ChairD_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_ChairD_01_collision.DAE
│  │  │  │  │  └─ aws_ChairD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Chandelier_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Chandelier_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Chandelier_01.DAE
│  │  │  │  │  ├─ aws_Chandelier_01_collision.DAE
│  │  │  │  │  └─ aws_Chandelier_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_CoffeeTable_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_CoffeeTable_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_CoffeeTable_01_collision.DAE
│  │  │  │  │  └─ aws_CoffeeTable_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_CookingBench_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_CookingBench_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_CookingBench_01.DAE
│  │  │  │  │  ├─ aws_CookingBench_01_collision.DAE
│  │  │  │  │  └─ aws_CookingBench_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Curtain_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Curtain_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Curtain_01_collision.DAE
│  │  │  │  │  └─ aws_Curtain_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitA_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitA_01.png
│  │  │  │  │     └─ aws_DeskPortraitA_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitA_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitA_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitA_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitA_01.png
│  │  │  │  │     └─ aws_DeskPortraitA_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitA_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitA_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitB_01.png
│  │  │  │  │     └─ aws_DeskPortraitB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitB_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitB_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitB_01.png
│  │  │  │  │     └─ aws_DeskPortraitB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitB_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitC_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitC_01.png
│  │  │  │  │     └─ aws_DeskPortraitC_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitC_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitC_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitC_01.png
│  │  │  │  │     └─ aws_DeskPortraitC_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitC_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitD_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitD_01.png
│  │  │  │  │     └─ aws_DeskPortraitD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitD_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitD_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitD_01.png
│  │  │  │  │     └─ aws_DeskPortraitD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitD_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitD_03
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitD_01.png
│  │  │  │  │     └─ aws_DeskPortraitD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitD_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_DeskPortraitD_04
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_DeskPortraitD_01.png
│  │  │  │  │     └─ aws_DeskPortraitD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_DeskPortraitD_01_collision.DAE
│  │  │  │  │  └─ aws_DeskPortraitD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Door_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Door_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Door_01_collision.DAE
│  │  │  │  │  └─ aws_Door_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Dumbbell_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Dumbbell_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Dumbbell_01_collision.DAE
│  │  │  │  │  └─ aws_Dumbbell_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_FitnessEquipment_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_FitnessEquipment_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_FitnessEquipment_01_collision.DAE
│  │  │  │  │  └─ aws_FitnessEquipment_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_FloorB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_FloorB_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_FloorB_01_collision.DAE
│  │  │  │  │  └─ aws_FloorB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_FoldingDoor_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_FoldingDoor_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_FoldingDoor_01_collision.DAE
│  │  │  │  │  └─ aws_FoldingDoor_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_FoldingDoor_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_FoldingDoor_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_FoldingDoor_02_collision.DAE
│  │  │  │  │  └─ aws_FoldingDoor_02_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Handle_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Handle_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_ handle_01_collision.DAE
│  │  │  │  │  ├─ aws_Handle_01_collision.DAE
│  │  │  │  │  └─ aws_Handle_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_HouseWallB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_HouseWallB_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_HouseWallB_01_collision.DAE
│  │  │  │  │  └─ aws_HouseWallB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_KitchenCabinet_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_KitchenCabinet_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_KitchenCabinet_01_collision.DAE
│  │  │  │  │  └─ aws_KitchenCabinet_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_KitchenTable_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_KitchenTable_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_KitchenTable_01_collision.DAE
│  │  │  │  │  └─ aws_KitchenTable_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_KitchenUtensils_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_KitchenUtensils_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_KitchenUtensils_01_collision.DAE
│  │  │  │  │  └─ aws_KitchenUtensils_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_LightC_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_LightC_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_LightC_01_collision.DAE
│  │  │  │  │  └─ aws_LightC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_NightStand_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_NightStand_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_NightStand_01_collision.DAE
│  │  │  │  │  └─ aws_NightStand_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Pillow_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Pillow_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Pillow_01_collision.DAE
│  │  │  │  │  └─ aws_Pillow_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitA_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitA_01.png
│  │  │  │  │     └─ aws_PortraitA_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitA_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitA_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitA_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitA_01.png
│  │  │  │  │     └─ aws_PortraitA_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitA_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitA_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitB_01.png
│  │  │  │  │     └─ aws_PortraitB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitB_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitB_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitB_01.png
│  │  │  │  │     └─ aws_PortraitB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitB_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitB_03
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitB_01.png
│  │  │  │  │     └─ aws_PortraitB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitB_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitC_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitC_01.png
│  │  │  │  │     └─ aws_PortraitC_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitC_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitD_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitD_01.png
│  │  │  │  │     └─ aws_PortraitD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitD_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitD_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitD_01.png
│  │  │  │  │     └─ aws_PortraitD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitD_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitE_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitE_01.png
│  │  │  │  │     └─ aws_PortraitE_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitE_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitE_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_PortraitE_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_PortraitE_01.png
│  │  │  │  │     └─ aws_PortraitE_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_PortraitE_01_collision.DAE
│  │  │  │  │  └─ aws_PortraitE_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Quilt_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Quilt_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Quilt_01_collision.DAE
│  │  │  │  │  └─ aws_Quilt_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Rangehood_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Rangehood_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Rangehood_01_collision.DAE
│  │  │  │  │  └─ aws_Rangehood_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_ReadingDesk_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_ReadingDesk_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_ReadingDesk_01_collision.DAE
│  │  │  │  │  └─ aws_ReadingDesk_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Refrigerator_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_Refrigerator_01.png
│  │  │  │  │     └─ aws_Refrigerator_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Refrigerator_01.DAE
│  │  │  │  │  ├─ aws_Refrigerator_01_collision.DAE
│  │  │  │  │  └─ aws_Refrigerator_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_RoomCeiling_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_RoomCeiling_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_RoomCeiling_01_collision.DAE
│  │  │  │  │  └─ aws_RoomCeiling_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_RoomWall_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_RoomWall_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_RoomWall_01_collision.DAE
│  │  │  │  │  └─ aws_RoomWall_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_RoomWindow_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_RoomWindow_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_RoomWindow_01_collision.DAE
│  │  │  │  │  └─ aws_RoomWindow_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_SeasoningBox_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_SeasoningBox_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_SeasoningBox_01_collision.DAE
│  │  │  │  │  └─ aws_SeasoningBox_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_SecurityCamera_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_SecurityCamera_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_SecurityCamera_01_collision.DAE
│  │  │  │  │  └─ aws_SecurityCamera_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_ShoeRack_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_ShoeRack_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_ShoeRack_01_collision.DAE
│  │  │  │  │  └─ aws_ShoeRack_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_SofaB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_SofaB_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_SofaB_01_collision.DAE
│  │  │  │  │  └─ aws_SofaB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_SofaC_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_SofaC_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_SofaC_01_collision.DAE
│  │  │  │  │  └─ aws_SofaC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Sofa_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Sofa_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Sofa_01.DAE
│  │  │  │  │  ├─ aws_Sofa_01_collision.DAE
│  │  │  │  │  └─ aws_Sofa_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_TVCabinet_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_TVCabinet_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_TVCabinet_01_collision.DAE
│  │  │  │  │  └─ aws_TVCabinet_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_TV_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_TV_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_TV_01_collision.DAE
│  │  │  │  │  └─ aws_TV_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_TV_02
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_TV_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_TV_01_collision.DAE
│  │  │  │  │  └─ aws_TV_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Tablet_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Tablet_01_nologo.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Tablet_01_collision.DAE
│  │  │  │  │  └─ aws_Tablet_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Tableware_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Tableware_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Tableware_01_collision.DAE
│  │  │  │  │  └─ aws_Tableware_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Trash_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Trash_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Trash_01_collision.DAE
│  │  │  │  │  └─ aws_Trash_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Vase_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Vase_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Vase_01_collision.DAE
│  │  │  │  │  └─ aws_Vase_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_residential_Wardrobe_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_Wardrobe_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_Wardrobe_01_collision.DAE
│  │  │  │  │  └─ aws_Wardrobe_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_Bucket_01
│  │  │  │  ├─ .DS_Store
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ .DS_Store
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_robomaker_warehouse_Bucket_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_Bucket_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_Bucket_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_ClutteringA_01
│  │  │  │  ├─ .DS_Store
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ .DS_Store
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ .DS_Store
│  │  │  │  │     ├─ aws_robomaker_warehouse_ClutteringA_01.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_ClutteringA_02.png
│  │  │  │  │     └─ aws_robomaker_warehouse_ClutteringA_03.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_ClutteringA_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_ClutteringA_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_ClutteringC_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_ClutteringC_01.png
│  │  │  │  │     └─ aws_robomaker_warehouse_ClutteringC_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_ClutteringC_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_ClutteringC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_ClutteringD_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_ClutteringD_01.png
│  │  │  │  │     └─ aws_robomaker_warehouse_ClutteringD_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_ClutteringD_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_ClutteringD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_DeskC_01
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ .DS_Store
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_DeskC_01.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_DeskC_02.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_DeskC_03.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_DeskC_03.psd
│  │  │  │  │     └─ aws_robomaker_warehouse_DeskC_04.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_DeskC_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_DeskC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_GroundB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_GroundB_01.png
│  │  │  │  │     └─ aws_robomaker_warehouse_GroundB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_GroundB_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_GroundB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_Lamp_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_robomaker_warehouse_Lamp_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_Lamp_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_Lamp_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_PalletJackB_01
│  │  │  │  ├─ .DS_Store
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ .DS_Store
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_PalletJackB_01.png
│  │  │  │  │     └─ aws_robomaker_warehouse_PalletJackB_02.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_PalletJackB_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_PalletJackB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_RoofB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_robomaker_warehouse_RoofB_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_RoofB_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_RoofB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_ShelfD_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfD_01.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfD_02.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfD_03.png
│  │  │  │  │     └─ aws_robomaker_warehouse_ShelfD_04.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_ShelfD_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_ShelfD_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_ShelfE_01
│  │  │  │  ├─ .DS_Store
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ .DS_Store
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfE_01.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfE_02.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfE_03.png
│  │  │  │  │     └─ aws_robomaker_warehouse_ShelfE_04.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_ShelfE_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_ShelfE_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_ShelfF_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfF_01.png
│  │  │  │  │     ├─ aws_robomaker_warehouse_ShelfF_02.png
│  │  │  │  │     └─ aws_robomaker_warehouse_ShelfF_03.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_ShelfF_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_ShelfF_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_TrashCanC_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_robomaker_warehouse_TrashCanC_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_TrashCanC_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_TrashCanC_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ aws_robomaker_warehouse_WallB_01
│  │  │  │  ├─ materials
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ aws_robomaker_warehouse_WallB_01.png
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ aws_robomaker_warehouse_WallB_01_collision.DAE
│  │  │  │  │  └─ aws_robomaker_warehouse_WallB_01_visual.DAE
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ building
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ chair_3
│  │  │  │  ├─ meshes
│  │  │  │  │  └─ model.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ chair_set_1
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  └─ chair_set_1.material
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ fabric_black.jpg
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ chair_cushions.dae
│  │  │  │  │  ├─ chair_frame.dae
│  │  │  │  │  ├─ chair_full.dae
│  │  │  │  │  ├─ model.dae
│  │  │  │  │  └─ sofa_cushions_back.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ coffee_table_set_1
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  ├─ coffee_table_set_1_frame.material
│  │  │  │  │  │  └─ coffee_table_set_1_surf.material
│  │  │  │  │  └─ textures
│  │  │  │  │     ├─ leather.jpg
│  │  │  │  │     └─ wood_dark_1.jpg
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ model.dae
│  │  │  │  │  ├─ table_large_frame.dae
│  │  │  │  │  ├─ table_large_full.dae
│  │  │  │  │  ├─ table_large_mid.dae
│  │  │  │  │  ├─ table_large_top.dae
│  │  │  │  │  └─ tables.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ desk_brown
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  └─ desk_brown.material
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ des_b.jpg
│  │  │  │  ├─ meshes
│  │  │  │  │  └─ table.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ door
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  └─ door.material
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ door.jpg
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ monitor_1
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  └─ monitor_screen.material
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ screen.jpg
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ frame.dae
│  │  │  │  │  ├─ monitor.dae
│  │  │  │  │  └─ screen1.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ office_chair
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ chair_base.dae
│  │  │  │  │  ├─ chair_cushion.dae
│  │  │  │  │  └─ chair_wheels.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ plant_2
│  │  │  │  ├─ meshes
│  │  │  │  │  └─ leaf2.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ pot_flower
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ full.dae
│  │  │  │  │  ├─ pot_base.dae
│  │  │  │  │  └─ pot_soil.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ table_breakfast
│  │  │  │  ├─ meshes
│  │  │  │  │  └─ table3.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  ├─ table_conference_1
│  │  │  │  ├─ materials
│  │  │  │  │  ├─ scripts
│  │  │  │  │  │  └─ table_conf_1.material
│  │  │  │  │  └─ textures
│  │  │  │  │     └─ table_grey.jpg
│  │  │  │  ├─ meshes
│  │  │  │  │  ├─ base.dae
│  │  │  │  │  ├─ model1.dae
│  │  │  │  │  └─ table.dae
│  │  │  │  ├─ model.config
│  │  │  │  └─ model.sdf
│  │  │  └─ vase_small
│  │  │     ├─ materials
│  │  │     │  ├─ scripts
│  │  │     │  │  └─ vase_pattern1.material
│  │  │     │  └─ textures
│  │  │     │     └─ bronze.jpg
│  │  │     ├─ meshes
│  │  │     │  └─ vase2.dae
│  │  │     ├─ model.config
│  │  │     └─ model.sdf
│  │  ├─ package.xml
│  │  ├─ patrolling_robot_simulation
│  │  │  ├─ __init__.py
│  │  │  └─ launch
│  │  │     └─ patrol_robot.launch.py
│  │  ├─ resource
│  │  │  └─ patrolling_robot_simulation
│  │  ├─ setup.cfg
│  │  ├─ setup.py
│  │  ├─ test
│  │  │  ├─ test_copyright.py
│  │  │  ├─ test_flake8.py
│  │  │  └─ test_pep257.py
│  │  └─ worlds
│  │     ├─ no_roof_small_warehouse.world
│  │     ├─ patrol_robot.world
│  │     └─ small_house.world
│  └─ yolo_object_detection
│     ├─ package.xml
│     ├─ resource
│     │  └─ yolo_object_detection
│     ├─ setup.cfg
│     ├─ setup.py
│     ├─ test
│     │  ├─ test_copyright.py
│     │  ├─ test_flake8.py
│     │  └─ test_pep257.py
│     └─ yolo_object_detection
│        ├─ __init__.py
│        ├─ __pycache__
│        │  ├─ __init__.cpython-310.pyc
│        │  ├─ yolo_image_saver_node.cpython-310.pyc
│        │  └─ yolo_object_detection_node.cpython-310.pyc
│        ├─ baseline_manager_node.py
│        ├─ launch
│        │  └─ yolo_detection_launch.py
│        ├─ patrolling_yolo_node.py
│        ├─ state_manager_node.py
│        ├─ waypoint_follower_node.py
│        └─ yolo_object_detection_node.py
└─ yolov8n.pt

```