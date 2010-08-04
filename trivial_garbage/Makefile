
PACKAGE_DIR=$(shell rospack find trivial_garbage)

all: installed

downloaded:
	darcs get http://common-lisp.net/~loliveira/darcs/trivial-garbage/
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ${PACKAGE_DIR}/trivial-garbage/trivial-garbage.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf trivial-garbage asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

