
REV_SHA_ID=7591d6dda666ac3b9353d3d38d0d394ab3a62688

all: installed

downloaded: Makefile
	make clean
	git clone git://repo.or.cz/gsll.git
	cd gsll; git checkout ${REV_SHA_ID} -b ros
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../gsll/gsll.asd asdf
	touch installed

clean:
	rm -rf gsll asdf
	rm -f downloaded rospack_nosubdirs installed patched

