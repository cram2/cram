
all: installed

downloaded: Makefile
	make clean
	darcs get http://common-lisp.net/project/cl-store/darcs/cl-store/
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	ln -s ../cl-store/cl-store.asd asdf/cl-store.asd
	touch installed

clean:
	rm -rf cl-store asdf downloaded rospack_no_subdirs installed

