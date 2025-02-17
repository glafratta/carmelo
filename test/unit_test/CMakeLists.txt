# PROJECT(test)
 
#cmake_minimum_required(VERSION 3.1.0)

find_package(OpenCV REQUIRED)

find_package(box2d REQUIRED)

find_package(PkgConfig)

find_package(Threads)

include(CTest)

INCLUDE_DIRECTORIES("/usr/local/include/")
enable_testing()

set(CTEST_OUTPUT_ON_FAILURE 1)

add_executable(split_task split_task.cpp)
target_link_libraries(split_task CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(backtrack backtrack.cpp)
target_link_libraries(backtrack CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(explorer explorer.cpp ../test_essentials.h)
target_link_libraries(explorer CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(transitionMatrix ../test_essentials.h transitionMatrix.cpp)
target_link_libraries(transitionMatrix CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(disturbance_dimensions ../test_essentials.h disturbance_dimensions.cpp)
target_link_libraries(disturbance_dimensions CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(planner ../test_essentials.h planner.cpp)
target_link_libraries(planner CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(split_step ../test_essentials.h split_step.cpp)
target_link_libraries(split_step CloCK ${OpenCV_LIBS} box2d rt  )
add_executable(affine_transformation ../test_essentials.h affine_transform.cpp)
target_link_libraries(affine_transformation CloCK ${OpenCV_LIBS} box2d rt  )



add_test(NAME split_task COMMAND split_task 0.6 0 0)
add_test(NAME split_task_-01 COMMAND split_task 0.26 -0.01 -1.570796 0 0 -1.570796 0.265 -0.16 0)
add_test(NAME split_task_angle COMMAND split_task 0.0 .6 1.57079633)
add_test(NAME split_task_offset COMMAND split_task 0.75 0.27 0 0.2 0.27 0)
add_test(NAME split_task_offset_angle COMMAND split_task 0.27 0.75 1.57079633 0.27 0.2 1.57079633)
add_test(NAME split_task_1_node COMMAND split_task 0.27 0 0)
add_test(NAME split_task_0_nodes COMMAND split_task 0.18 0 0)

add_test(NAME split_step COMMAND split_step 0.68 0 )

add_test(NAME backtrack COMMAND backtrack 0.6 0 0)
add_test(NAME backtrack_-01 COMMAND backtrack 0.26 -0.01 -1.570796 0 0 -1.570796 0.265 -0.16 0)

add_test(NAME transition COMMAND transitionMatrix)
add_test(NAME transition_n_L_ok COMMAND transitionMatrix 0 l k)
add_test(NAME transition_n_un_ok COMMAND transitionMatrix 0 u)
add_test(NAME transition_n_D_crash COMMAND transitionMatrix 0 d c)
add_test(NAME transition_n_D_ok COMMAND transitionMatrix 0 d k)
add_test(NAME transition_n_D_safe COMMAND transitionMatrix 0 d s)
add_test(NAME transition_y_R_behind COMMAND transitionMatrix 1 r k -1 0 -3.14)
add_test(NAME transition_y_D_crash COMMAND transitionMatrix y d c)
add_test(NAME transition_y_D_ok COMMAND transitionMatrix y d k)
add_test(NAME transition_y_D_safe COMMAND transitionMatrix y d s)
add_test(NAME transition_y_stop_crash COMMAND transitionMatrix y s c)
add_test(NAME transition_y_stop_ok COMMAND transitionMatrix y s k)
add_test(NAME transition_y_stop_safe COMMAND transitionMatrix y s s)
add_executable(processData_k ../../src/worldbuilder.h ../../src/worldbuilder.cpp processData_k.cpp)
add_executable(kmeans_clusters ../../src/worldbuilder.h ../../src/worldbuilder.cpp kmeans_clusters.cpp)
target_link_libraries(processData_k CloCK ${OpenCV_LIBS} box2d rt)
target_link_libraries(kmeans_clusters CloCK ${OpenCV_LIBS} box2d rt)

add_test(NAME affine_transformation_ahead COMMAND affine_transformation 1 0 0)

# # add_executable(split_task split_task.cpp)
# # target_link_libraries(split_task CloCK ${OpenCV_LIBS} box2d rt  )
# # add_executable(backtrack backtrack.cpp)
# # target_link_libraries(backtrack CloCK ${OpenCV_LIBS} box2d rt  )
# add_executable(explorer ../callbacks.h explorer.cpp )
# # add_executable(transitionMatrix ../callbacks.h transitionMatrix.cpp)
# # target_link_libraries(transitionMatrix CloCK ${OpenCV_LIBS} box2d rt  )
# add_executable(disturbance_dimensions ../callbacks.h disturbance_dimensions.cpp)
# add_executable(planner ../callbacks.h planner.cpp)
add_executable(match ../test_essentials.h match.cpp)
add_executable(find_match_state ../test_essentials.h find_match_s.cpp)
add_executable(state_difference ../test_essentials.h state_difference.cpp)
add_executable(match_equal ../test_essentials.h match_equal.cpp)
add_executable(replanner ../test_essentials.h replanner.cpp)
add_executable(fill_for_dist ../test_essentials.h fill_for_dist.cpp)
add_executable(make_sensor ../test_essentials.h make_sensor.cpp)
add_executable(end_contact ../test_essentials.h end_contact.cpp)
add_executable(get_disturbance ../test_essentials.h get_disturbance.cpp)
add_executable(checkPlan ../test_essentials.h checkPlan.cpp)

target_link_libraries(match CloCK ${OpenCV_LIBS} box2d )
target_link_libraries(find_match_state CloCK ${OpenCV_LIBS} box2d )
target_link_libraries(state_difference CloCK ${OpenCV_LIBS} box2d )
target_link_libraries(match_equal CloCK ${OpenCV_LIBS} box2d )
target_link_libraries(replanner CloCK ${OpenCV_LIBS} box2d )
target_link_libraries(fill_for_dist CloCK ${OpenCV_LIBS} box2d)
target_link_libraries(make_sensor CloCK ${OpenCV_LIBS} box2d)
target_link_libraries(end_contact CloCK ${OpenCV_LIBS} box2d)
target_link_libraries(get_disturbance CloCK ${OpenCV_LIBS} box2d)
target_link_libraries(checkPlan CloCK ${OpenCV_LIBS} box2d )

add_test(NAME processData_k COMMAND processData_k ../170924_1321_badrun/ 1 )

add_test(NAME kmeans_clusters COMMAND kmeans_clusters ../170924_1321_badrun/ 0 )


add_test(NAME fill_avoid1 COMMAND fill_for_dist ../251023_TAGET/ 0.37 0 1.5714 1)
add_test(NAME fill_avoid2 COMMAND fill_for_dist ../251023_TAGET/ 0.37 0.27 0 2)
add_test(NAME fill_avoid_close COMMAND fill_for_dist ../251023_TAGET/ 0.54 0 1.5714 3)
add_test(NAME make_sensor_start COMMAND make_sensor 0 0 0 6)
add_test(NAME make_sensor_turned COMMAND make_sensor 0 0 1.5714 7)
add_test(NAME make_sensor_close COMMAND make_sensor 0.54 0 1.5714 4)
add_test(NAME make_sensor_far COMMAND make_sensor 0.20 0 1.5714 5)

add_test(NAME end_contact_obstacle_avoid COMMAND end_contact 0 0)
add_test(NAME end_contact_obstacle_goal COMMAND end_contact 0 1)
add_test(NAME end_contact_goal COMMAND end_contact 1 1)

add_test(NAME get_disturbance_0dl COMMAND get_disturbance 0 2 0) #position 1, prev: DEF, curr: LEFT
add_test(NAME get_disturbance_1l COMMAND get_disturbance 1 0) #position 1, prev: LEFT, curr: DEF
add_test(NAME get_disturbance_2r COMMAND get_disturbance 2 1) #position 2, prev: RIGHT, curr:DEF
add_test(NAME get_disturbance_3 COMMAND get_disturbance 3)
add_test(NAME get_disturbance_3dr COMMAND get_disturbance 3 2 1)
add_test(NAME get_disturbance_3l COMMAND get_disturbance 3 0)
add_test(NAME get_disturbance_3r COMMAND get_disturbance 3 1)
add_test(NAME get_disturbance_4r COMMAND get_disturbance 4)
add_test(NAME get_disturbance_4dr COMMAND get_disturbance 4 2 1)

add_test(NAME disturbance_dimensions_1d COMMAND disturbance_dimensions)
#add_test(NAME disturbance_dimensions_2d COMMAND disturbance_dimensions 2)


add_test(NAME explorer_no COMMAND explorer)
add_test(NAME explorer_cds COMMAND explorer ../cul_de_sac/ 0)
add_test(NAME explorer_target COMMAND explorer ../251023_TAGET/ 1)


add_test(NAME planner_no COMMAND planner)
add_test(NAME planner_cds COMMAND planner ../cul_de_sac/ 0)
add_test(NAME planner_target_40cm COMMAND planner ../170924_1321_badrun/ 1)
add_test(NAME planner_target_68cm COMMAND planner ../251023_TAGET 1)


add_test(NAME match_false COMMAND match)
add_test(NAME match_true_exact COMMAND match 1)
add_test(NAME match_abstract COMMAND match 6)
add_test(NAME match_Dn_no COMMAND match 2 )
add_test(NAME match_Dn_ratio COMMAND match 2 1)
add_test(NAME match_Dn_position COMMAND match 2 2)
set_property(TEST match_Dn_no PROPERTY WILL_FAIL true)
set_property(TEST match_Dn_position PROPERTY WILL_FAIL true)
add_test(NAME match_Dn_shape COMMAND match 2 3)
set_property(TEST match_Dn_shape PROPERTY WILL_FAIL true)

add_test(NAME match_Di_no COMMAND match 5 )
add_test(NAME match_Di_ratio COMMAND match 5 1)
add_test(NAME match_Di_position COMMAND match 5 2)
set_property(TEST match_Di_no PROPERTY WILL_FAIL true)
set_property(TEST match_Di_position PROPERTY WILL_FAIL true)
add_test(NAME match_Di_shape COMMAND match 5 3)
set_property(TEST match_Di_shape PROPERTY WILL_FAIL true)


add_test(NAME find_match_state COMMAND find_match_state ../251023_TAGET/ 2 1)
add_test(NAME find_match_state_f COMMAND find_match_state ../251023_TAGET/ 3 1)
add_test(NAME find_match_3 COMMAND find_match_state ../251023_TAGET/ 3 1 29)
add_test(NAME find_match_midtask11 COMMAND find_match_state ../251023_TAGET/ 3 1 14)
add_test(NAME find_match_midtask2 COMMAND find_match_state ../251023_TAGET/ 2 1 14)
add_test(NAME find_match_quarter COMMAND find_match_state ../251023_TAGET/ 2 1 7)

set_property(TEST find_match_midtask11 PROPERTY WILL_FAIL true)
set_property(TEST find_match_state_f PROPERTY WILL_FAIL true)

add_test(NAME find_match_state_turn COMMAND find_match_state ../251023_TAGET/ 9 1 55)

add_test(NAME state_difference COMMAND state_difference ../251023_TAGET/ 1)
add_test(NAME state_difference_turn COMMAND state_difference ../251023_TAGET/ 1 55)

add_test(NAME match_equal COMMAND match_equal)
#add_test(NAME replanner_40 COMMAND replanner ../170924_1321_badrun/ 2 1)
add_test(NAME replanner_68 COMMAND replanner ../251023_TAGET/ 2 1)
add_test(NAME replanner_empty COMMAND replanner empty 2 1)
add_test(NAME replanner_back COMMAND replanner empty 2 1 100 back)



# add_test(NAME replanner_68_9 COMMAND replanner ../251023_TAGET/ 9 1 34)
# add_test(NAME replanner_68_11 COMMAND replanner ../251023_TAGET/ 11 1 34)
# add_test(NAME replanner_68_10 COMMAND replanner ../251023_TAGET/ 9 1 50)
# add_test(NAME replanner_68_12 COMMAND replanner ../251023_TAGET/ 11 1 50)


add_test(NAME checkPlan COMMAND checkPlan ../251023_TAGET/ 2 1 1)
add_test(NAME checkPlan_4 COMMAND checkPlan ../251023_TAGET/ 2 1 4)
add_test(NAME checkPlan_21 COMMAND checkPlan ../251023_TAGET/ 2 1 21)
add_test(NAME checkPlan_40cm COMMAND checkPlan ../170924_1321_badrun/ 2 1 1)
add_test(NAME checkPlan_40cm_4 COMMAND checkPlan ../170924_1321_badrun/ 2 1 4)
add_test(NAME checkPlan_40cm_21 COMMAND checkPlan ../170924_1321_badrun/ 2 1 21)

#set_property(TEST checkPlan_21 PROPERTY WILL_FAIL true)


add_executable(subgraph ../test_essentials.h subgraph.cpp)
target_link_libraries(subgraph CloCK ${OpenCV_LIBS} box2d rt)
add_test(NAME subgraph COMMAND subgraph)


set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-Wall -pedantic)

