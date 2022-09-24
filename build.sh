#! /bin/sh

branch="master"
tag="latest"

print_usage() {
  printf "Usage: 
  docker tag: -t [tag](default: latest)
  branch to use for nodes: -b [path](default: master)
"
}

build_docker() {
  printf "building ghcr.io/priestofadanos/lc_sensor_grabber:$tag... \n"
  docker build --no-cache --build-arg BRANCH=$branch  --tag ghcr.io/priestofadanos/lc_sensor_grabber:$tag --file  docker/Dockerfile docker/
}

while getopts "t:b:" flag; do
  case "${flag}" in
    t) tag="${OPTARG}" 
       print_help=false
       ;;
    b) branch="${OPTARG}"
       print_help=false
       ;;
    *) print_usage
       print_help=false 
       exit 1
       ;;
  esac
done


if [ "$print_help" = true ] ; 
  then
      print_usage
  else
      run_docker
fi




