cmake_minimum_required(VERSION 3.22)
project(panda_servo_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

ament_package()

set(SCRIPTS_DIR scripts)
install(PROGRAMS
    ${SCRIPTS_DIR}/evaluate.py
    ${SCRIPTS_DIR}/preload_replay_buffer.py
    ${SCRIPTS_DIR}/random_agent.py
    ${SCRIPTS_DIR}/train.py
    ${SCRIPTS_DIR}/train_dreamerv2.py
    ${SCRIPTS_DIR}/utils/dataset/dataset_download_test.bash
    ${SCRIPTS_DIR}/utils/dataset/dataset_download_train.bash
    ${SCRIPTS_DIR}/utils/dataset/dataset_set_test.bash
    ${SCRIPTS_DIR}/utils/dataset/dataset_set_train.bash
    ${SCRIPTS_DIR}/utils/process_collection.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
