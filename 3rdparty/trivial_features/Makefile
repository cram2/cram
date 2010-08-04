
PACKAGE_DIR=$(shell rospack find trivial_features)

all: installed

downloaded:
	darcs get http://common-lisp.net/~loliveira/darcs/trivial-features
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ${PACKAGE_DIR}/trivial-features/trivial-features.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf trivial-features asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

