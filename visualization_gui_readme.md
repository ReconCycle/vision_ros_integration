# VISUALIZATION GUI <br/>
Program is designed to help user with visualization of robot cell in RVIZ. It has two basic functionalities - scan network to detect connected tables <br/>
and load already created yaml file which describes cell configuration. On the start-up, gui window contains only two buttons: "SCAN NETWORK" and <br/>
"LOAD CONFIGURATION". After either of those two buttons is selected, gui will change a little: those two buttons will move on top of gui screen and on bottom four additional buttons <br/>
will show up: "ADD TABLE", "REMOVE TABLE", "SAVE YAML" and "RVIZ". <br/>
## SCAN NETWORK <br/>
When "SCAN NETWORK" button is pressed, program starts with searching in pre-defined range of IPs. If IP is connected, program reads its hostname and than shows it in gui window. <br/>
Additional filter for filtering hostnames will be added. All detected hostnames are than displayed in gui window, each one in its own row. Next to hostnames are four text-input blocks <br/>
and one checkbox. First text-input block is for entering x-position of table, second is for y-position, third for rotation in degrees and fourth for stl file name (functionality of <br/>
stl file name text-input will be changed asap, so it is not yet very important part of the program). User should enter discrete values for x and y position. <br/>
### ENTERING X AND Y POSITIONS <br/>
Base table should have x and y-inputs equal to 0, so it will be placed in the origin of world frame in RVIZ. Table next to it on x-axis should have x-input equal to 1 and y-input <br/>
equal to 0. Table next to base table on y-axis should have x-input equal to 0 and y-input equal to 1 and so on. Basically, x and y position of table should be equal to how many tables <br/>
away (in x or y direction) is table from base table. <br/>
## ADD OR REMOVE TABLE </br>
User can add additional, non-connected table with click on "ADD TABLE" button. First, a popup with text-input block will open and user will be asked to enter a table-name. Name of the </br>
additional table should be different than all others names or hostnames. After entering a valid table-name, a new row with table name and text-input blocks will be added in gui window.<br/>
User can also remove table by first checking the corresponding checkbox and than click on "REMOVE TABLE" button. <br/>
## SAVE YAML </br>
When user is satisfied with entered inputs, he/she can save configuration in yaml file by clicking on "SAVE YAML" button. A popup with text-input block will appear and user will be <br/>
asked to enter a yaml filename. Yaml file will be saved in config/cell_config directory. br/>
## RVIZ <br/>
By clicking on "RVIZ" button, entered configuration will be visualized in RVIZ. By clicking on this button, RVIZ will also refresh rviz and possible changes in configuration <br/>
will be visualized. <br/>
## LOAD CONFIGURATION <br/>
Instead of scanning the network for connected tables each time, user can also load already saved configuration (yaml file). Gui window will be populated with hostnames (table-names) and <br/>
text-inputs, but those will already contain saved values. User can change those values, add additional tables or remove them, etc. <br/>
## OPEN THE GUI <br/>
Gui can be opened by running robot_cell_launch.launch file with roslaunch command. This launch file will open an empty rviz configuration and gui in novnc on localhost:8080.
## DOCKER
Gui is written in Python with Kivy library. Dockerfile is placed in reconcycle_dockers/docker_files/reconcycle-cell-visualization in branch devel. Docker-compose file is placed in <br/>
reconcycle_dockers/compose-files/reconcycle-layout-manager in same branch.
