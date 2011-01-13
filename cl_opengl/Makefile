
all: installed

downloaded: Makefile
	make clean
	git clone https://github.com/3b/cl-opengl.git
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../cl-opengl/cl-opengl.asd asdf/
	ln -s ../cl-opengl/cl-glu.asd asdf/
	ln -s ../cl-opengl/cl-glut.asd asdf/
	ln -s ../cl-opengl/cl-glut-examples.asd asdf/
	touch installed

clean:
	rm -rf cl-opengl asdf
	rm -f downloaded rospack_nosubdirs installed patched
