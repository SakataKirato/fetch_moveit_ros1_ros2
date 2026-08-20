# ----------------------------------------------------------------------
#  Docker Development
# ----------------------------------------------------------------------

#: Builds a Docker image with the corresponding Dockerfile file

# ---------NOETIC----------

noetic.build:
	@./docker/scripts/build.bash --ros-distro=noetic

noetic.build.cuda:
	@ ./docker/scripts/build.bash --ros-distro=noetic --use-cuda --cuda-image=$(cuda-image) --cuda-version=$(cuda-version)


# ----------------------------CREATE------------------------------------

# Create containers with Noetic
noetic.create:
	@./docker/scripts/run.bash --ros-distro=noetic $(if $(volumes),--volumes=$(volumes)) $(if $(name),--name=$(name))

noetic.create.cuda:  
	@./docker/scripts/run.bash --ros-distro=noetic --use-cuda $(if $(volumes),--volumes=$(volumes)) $(if $(name),--name=$(name))

# ----------------------------START------------------------------------
# Start containers
noetic.up:
	@ if [ -n "$(DISPLAY)" ]; then xhost +; fi
	@docker start ros-noetic-$(USER)

# ----------------------------STOP------------------------------------
# Stop containers
noetic.down:
	@docker stop ros-noetic-$(USER)

# ----------------------------RESTART------------------------------------
# Restart containers
noetic.restart:
	@docker restart ros-noetic-$(USER)

# ----------------------------LOGS------------------------------------
# Logs of the container
noetic.logs:
	@docker logs --tail 50 ros-noetic-$(USER)

# ----------------------------SHELL------------------------------------
# Fires up a bash session inside the container
noetic.shell:
	@docker exec -it --user $(shell id -u):$(shell id -g) ros-noetic-$(USER) bash 

# ----------------------------REMOVE------------------------------------
# Remove container
noetic.remove:
	@docker container rm ros-noetic-$(USER)

# ----------------------------------------------------------------------
#  General Docker Utilities

#: Show a list of images.
list-images:
	@docker image ls

#: Show a list of containers.
list-containers:
	@docker container ls -a
