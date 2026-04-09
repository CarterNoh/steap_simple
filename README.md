# steap_simple
Implementation of STEAP algorithm for simple 2D example



INSTRUCTIONS
0. Clone repository
2. Create "output" folder in repository directory
2. From repository, build docker image from docker file using __
3. Run docker image, mounting drives appropriately using "docker run --rm -v $(pwd):/steap -v $(pwd)/output:/steap/output steapsimple:latest"
   (or for windows, docker run --rm -v ${PWD}:/steap -v ${PWD}/output:/steap/output steapsimple:latest)
