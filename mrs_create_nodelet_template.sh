#!/bin/bash

# set -o xtrace
set -o errexit
set -o nounset
set -o pipefail

PROJECT_NAME=''
NAMESPACE_NAME=''
CLASS_NAME=''
AUTHOR_NAME='todo_name'
AUTHOR_EMAIL='todo@email.com'
PROJECT_DESCRIPTION=''

CMAKE_PACKAGES=()

help() {
    echo "
  Usage: long form   ./mrs_create_nodelet_template.sh [options]
  OR:    short form  ./mrs_create_nodelet_template.sh -s project_name namespace_name class_name path
  Options:
    -h      --help                  show help message
    -pn     --project-name          project name. example_project by default
    -pd     --project-description   project description. Write description as a text in quotes
    -cp     --cmake-packages        Used for cmake \"find_package\". default: 'roscpp mrs_lib std_msgs'
    -nn     --namespace-name        namespace
    -cn     --class-name            nodelet class name
    -an     --author-name           author name
    -ae     --author-email          should be specified for correct compilation!
    -pp     --project-path          where to create new project. default: ./ 
    "
    exit 0
}

if [[ $1 == "-s" ]]; then
  PROJECT_NAME=$2
  NAMESPACE_NAME=$3
  CLASS_NAME=$4
  PROJECT_PATH=$5
else
  while [ $# -ge 1 ]; do
    case "$1" in
    -pn | --project-name)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        PROJECT_NAME="$2"
      else
        echo "Error: wrong value of $1"
        exit 1
      fi
      shift 2
      ;;
    -nn | --namespace-name)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        NAMESPACE_NAME="$2"
      else
        echo "Error: wrong value of $1"
        exit 1
      fi
      shift 2
      ;;
    -cn | --class-name)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        CLASS_NAME="$2"
      else 
        echo "Error: wrong value of $1"
        exit 1
      fi
      shift 2
      ;;
    -an | --author-name)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        AUTHOR_NAME="$2"
      else
        echo "Error: wrong value of $1"
        exit 1
      fi
      shift 2
      ;;
    -pp | --project-path)
       if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        PROJECT_PATH="$2"
      else
        echo "Error: wrong value of $1"
        exit 1
      fi
      shift 2
      ;;
    -ae | --author-email)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        AUTHOR_EMAIL="$2"
      else
        echo "Error: wrong value of $1"
        exit 1
      fi
      shift 2
      ;;
    -pd | --project-description)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        PROJECT_DESCRIPTION="$2"
      else
        echo "Error: wrong name of $1"
        exit 1
      fi
      shift 2
      ;;
    -cp | --cmake-package)
      if [ $# -ge 2 ] && [[ ! $2 =~ ^-.*$ ]] ; then
        CMAKE_PACKAGES+=($2)
      else
        echo "Error: wrong name of $1"
        exit 1
      fi
      shift 2
      ;;
    -h | --help)
      help
      ;;
    -.*)
      echo "Invalid option: $1" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires arguments." >&2
      exit 1
      ;;
    *)
      if [ $# -ge 0 ] ; then
        echo "Error: wrong parameters cpecified: $1"
        echo $1
        help
        exit 1
      else
        break
      fi
      ;;
    esac
  done
fi

  

if [ -z "$PROJECT_NAME" ]; then 
  echo "Project name should be cpecified";
  help;
  exit 1; 
fi

if [ -z "$NAMESPACE_NAME" ]; then 
  echo "Namespace name should be cpecified";
  help;
  exit 1; 
fi

if [ -z "$CLASS_NAME" ]; then 
  echo "Class name (nodelet name) should be cpecified";
  help;
  exit 1; 
fi

if [ -z "$AUTHOR_EMAIL" ]; then 
  echo "Author email should be cpecified";
  help;
  exit 1; 
fi



printf -v CMAKE_PACKAGES "%s " "${CMAKE_PACKAGES[@]}"

CMAKE_PACKAGES=${CMAKE_PACKAGES%?}

git clone git@github.com:Myralllka/template_nodelet.git "$PROJECT_NAME"

(
  cd "$PROJECT_NAME" 
  
  sed -i "s/more_project_name/$PROJECT_NAME/g" ./CMakeLists.txt || exit 1
  sed -i "s/more_filesname/$CLASS_NAME/g" ./CMakeLists.txt || exit 1
  sed -i "s/more_cmake_packages/$CMAKE_PACKAGES/g" ./CMakeLists.txt || exit 1
  echo -e "CMakeLists.txt done"

  sed -i "s/more_project_name/$PROJECT_NAME/g" ./plugins.xml || exit 1
  sed -i "s/more_class_name/$CLASS_NAME/g" ./plugins.xml || exit 1
  sed -i "s/more_namespace_name/$NAMESPACE_NAME/g" ./plugins.xml || exit 1
  echo -e "plugins.xml done"

  sed -i "s/more_email/$AUTHOR_EMAIL/g" ./package.xml || exit 1
  sed -i "s/more_name/$AUTHOR_NAME/g" ./package.xml || exit 1
  sed -i "s/more_project_name/$PROJECT_NAME/g" ./package.xml || exit 1
  echo -e "package.xml done"

  mv ./include/example.h "./include/$CLASS_NAME.h" || exit 1
  sed -i "s/CLASS_NAME/$CLASS_NAME/g" "./include/$CLASS_NAME.h" || exit 1
  sed -i "s/NAMESPACE_NAME/$NAMESPACE_NAME/g" "./include/$CLASS_NAME.h" || exit 1
  echo -e "creating $CLASS_NAME.h done"

  mv ./src/example.cpp "./src/$CLASS_NAME.cpp" || exit 1
  sed -i "s/CLASS_NAME/$CLASS_NAME/g" "./src/$CLASS_NAME.cpp" || exit 1
  sed -i "s/NAMESPACE_NAME/$NAMESPACE_NAME/g" "./src/$CLASS_NAME.cpp" || exit
  echo -e "creating $CLASS_NAME.cpp done"

  mv ./launch/example.launch "./launch/$PROJECT_NAME.launch" || exit 1
  sed -i "s/CLASS_NAME/$CLASS_NAME/g" "./launch/$PROJECT_NAME.launch" || exit 1
  sed -i "s/NAMESPACE_NAME/$NAMESPACE_NAME/g" "./launch/$PROJECT_NAME.launch" || exit 1
  sed -i "s/PROJECT_NAME/$PROJECT_NAME/g" "./launch/$PROJECT_NAME.launch" || exit 1
  echo -e "creating $CLASS_NAME.launch done\n"

  rm -rf .git || exit 1
  echo "Creating empty $PROJECT_NAME project finished. Now you can rename this directory, add it to git and start working."
)


