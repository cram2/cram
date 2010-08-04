
PACKAGE_DIR=$(shell rospack find babel)

all: installed

downloaded:
	darcs get http://common-lisp.net/project/babel/darcs/babel
	mkdir -p asdf
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	ln -s ${PACKAGE_DIR}/babel/babel.asd ${PACKAGE_DIR}/asdf/babel.asd
	ln -s ${PACKAGE_DIR}/babel/babel-streams.asd ${PACKAGE_DIR}/asdf/babel-streams.asd
	touch installed

clean:
	rm -rf babel asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

