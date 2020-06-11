#!/bin/sh

# Script modified from scripts by Jeroen de Bruijn, thephez, and Adafruit
# https://gist.github.com/vidavidorra/548ffbcdae99d752da02
# https://github.com/thephez/doxygen-travis-build
# https://learn.adafruit.com/the-well-automated-arduino-library/travis-ci

# Exit with nonzero exit code if anything fails
set -e

# Create a clean working directory for this script.
mkdir $TRAVIS_BUILD_DIR/code_docs
cd $TRAVIS_BUILD_DIR/code_docs
echo "Documentation path: " $(pwd)

# Re-clone the main repo, not sparsely
git clone -b master https://github.com/EnviroDIY/ModularSensors ModularSensors

# Clone the wiki, because we'll be using it in the docs
git clone https://github.com/EnviroDIY/ModularSensors.wiki

# Get the current gh-pages branch
# git clone -b gh-pages https://git@$GH_REPO_REF
# cd $GH_REPO_NAME
git clone -b gh-pages https://github.com/EnviroDIY/ModularSensors ModularSensorsDoxygen
cd ModularSensorsDoxygen

##### Configure git.
# Set the push default to simple i.e. push only the current branch.

# Remove everything currently in the gh-pages branch.
# GitHub is smart enough to know which files have changed and which files have
# stayed the same and will only update the changed files. So the gh-pages branch
# can be safely cleaned, and it is sure that everything pushed later is the new
# documentation.
rm -rf *

# Need to create a .nojekyll file to allow filenames starting with an underscore
# to be seen on the gh-pages site. Therefore creating an empty .nojekyll file.
# Presumably this is only needed when the SHORT_NAMES option in Doxygen is set
# to NO, which it is by default. So creating the file just in case.
echo "" > .nojekyll