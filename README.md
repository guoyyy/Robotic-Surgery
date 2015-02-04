# Robotic-Surgery
This project contains two sub-projects: RobotGUI and RobotSoftware

RobotGUI is a graphical user interface for Small Animal Biopsy Robot (SABiR http://www.ncbi.nlm.nih.gov/pmc/articles/PMC2796956/).
The GUI is based on java swing library and jogl (openGL), Kumiko Sano and  Sean Kruer implement the openGL part.

RobotSoftware is a software architecture of SABiR. 
src folder is the source code of robtic trajectory generation and data collection. Collected data is stored in raw_trajectory folder.

PD_simulator contains robot API for matlab. The simulink model is implemented by Mark Renfrew
