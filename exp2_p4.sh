bsn=$PWD

gnome-terminal -e roscore & sleep 3s

#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch probe.launch' & sleep 1s
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch effector.launch' & sleep 1s
#
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g4t1.launch' & sleep 5s
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g3t1_1.launch'
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g3t1_2.launch'
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g3t1_3.launch'
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/environment   -e 'roslaunch patient.launch'  & sleep 120s
#
#rosnode kill -a & sleep 5s
#
#mv -f ${bsn}/src/diagnostics/logs/p4/centralhub/*.log ${bsn}/src/diagnostics/logs/p4/centralhub/noiseless/
#
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P4 -e 'roslaunch P4_analyzer.launch'

gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/target_system -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/environment   -e 'roslaunch patient.launch'  & sleep 300s

################# SIMULATION #################
#gnome-terminal --working-directory=${bsn}/configurations/experiments/p4/simulation -e 'roslaunch injector.launch' & sleep 60s

rosnode kill -a
