
REV_SHA_ID=a79e13b4faf34ae9d69fcabc5def6229cb91efc2

all: installed

downloaded: Makefile
	make clean
	git clone git://repo.or.cz/gsd.git
	cd gsd; git checkout ${REV_SHA_ID} -b ros
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../gsd/foreign-array.asd ../gsd/grid.asd asdf
	touch installed

clean:
	rm -rf gsd asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

