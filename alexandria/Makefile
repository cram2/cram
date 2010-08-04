
PACKAGE_DIR=$(shell rospack find alexandria)

all: installed

downloaded: Makefile
	make clean
	darcs get http://common-lisp.net/project/alexandria/darcs/alexandria
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	ln -s ${PACKAGE_DIR}/alexandria/alexandria.asd ${PACKAGE_DIR}/asdf/alexandria.asd
	touch installed

clean:
	rm -rf alexandria asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

