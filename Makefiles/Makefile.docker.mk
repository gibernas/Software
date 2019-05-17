
docker:
	@echo "$(sep)Docker commands"
	@echo
	@echo 'For using Docker images'
	@echo
	@echo '- `make docker-build`:    Creates the image.'
	@echo '- `make docker-upload`:   Uploads the image.'
	@echo '- `make docker-clean`:    Removes all local images.'
	@echo
	@echo


#docker_dir=.circleci/images/duckietown-xenial-kinetic/
#docker_image_name=andreacensi/duckietown-xenial-kinetic
#tag=20
#
#docker-build:
#	docker build -t $(docker_image_name):$(tag) $(docker_dir)
#
#docker-upload:
#	docker push $(docker_image_name):$(tag)

branch=$(shell git rev-parse --abbrev-ref HEAD)
docker_image_name=gibernas/rpi-duckiebot-base:$(branch)

docker-build:
	docker build --pull -t $(docker_image_name) .

docker-upload:
	docker push $(docker_image_name)

docker-build-no-cache:
	docker build --no-cache -t $(docker_image_name) .

#docker-clean:
#	# Kill all running containers:
#	-sudo docker kill $(shell sudo docker ps -q)
#
#	# Delete all stopped containers (including data-only containers):
#	-sudo docker rm $(shell sudo docker ps -a -q)
#
#	# Delete all exited containers
#	-sudo docker rm $(shell sudo docker ps -q -f status=exited)
#
#	# Delete all images
#	-sudo docker rmi $(shell sudo docker images -q)
