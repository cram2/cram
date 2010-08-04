
PACKAGE_DIR=$(shell rospack find gsd)
REV_SHA_ID=a79e13b4faf34ae9d69fcabc5def6229cb91efc2

all: installed

downloaded:
	git clone git://repo.or.cz/gsd.git
	cd gsd; git checkout ${REV_SHA_ID} -b ros
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ${PACKAGE_DIR}/gsd/foreign-array.asd ${PACKAGE_DIR}/gsd/grid.asd ${PACKAGE_DIR}/asdf
	touch installed

clean:
	rm -rf gsd asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

