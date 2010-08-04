
PACKAGE_DIR=$(shell rospack find cl_store)

all: installed

downloaded:
	darcs get http://common-lisp.net/project/cl-store/darcs/cl-store/
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	ln -s ${PACKAGE_DIR}/cl-store/cl-store.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf cl-store asdf downloaded rospack_no_subdirs installed

