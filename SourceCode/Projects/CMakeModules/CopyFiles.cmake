# Copies all files in folder spcified
message(STATUS "Copy files in folder: ${Folder} to ${Destination}")
file(GLOB Files "${Folder}/*.*")
file(COPY ${Files} DESTINATION ${Destination})