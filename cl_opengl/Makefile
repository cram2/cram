
PACKAGE_DIR=$(shell rospack find cl_opengl)

all: installed

downloaded: Makefile
	make clean
	git clone https://github.com/3b/cl-opengl.git
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p ${PACKAGE_DIR}/asdf
	ln -s ${PACKAGE_DIR}/cl-opengl/cl-opengl.asd ${PACKAGE_DIR}/asdf
	ln -s ${PACKAGE_DIR}/cl-opengl/cl-glu.asd ${PACKAGE_DIR}/asdf
	ln -s ${PACKAGE_DIR}/cl-opengl/cl-glut.asd ${PACKAGE_DIR}/asdf
	ln -s ${PACKAGE_DIR}/cl-opengl/cl-glut-examples.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf cl-opengl asdf
	rm -f downloaded rospack_nosubdirs installed patched
