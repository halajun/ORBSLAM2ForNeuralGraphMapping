# ORB-SLAM2 For Neural Graph Mapping

This repostory is originally from [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) repo, and is served as the graph SLAM solution for our [neural graph mapping](https://github.com/roym899/neural_graph_mapping) algorithm. 

Several datasets (including scannet, replica, replica_big, neural RGBD, TUM and kintinuous) have been tested using this SLAM solution, and for each data sequence a pose graph that contains estimated poses is saved for our mapping method. The source (.cc) and config (.yaml) files for all tested datasets are in the [Example/RGB-D](https://github.com/halajun/ORBSLAM2ForNeuralGraphMapping/tree/main/Examples/RGB-D) folder. Example commands to execute the SLAM method are shown in the [execute_commands.txt](https://github.com/halajun/ORBSLAM2ForNeuralGraphMapping/blob/main/execute_commands.txt).

