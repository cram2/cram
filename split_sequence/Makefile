
SPLIT_SEQ_ARCHIVE=split-sequence.tar.gz

all: installed

downloaded: Makefile
	make clean
	wget http://code.in.tum.de/files/$(SPLIT_SEQ_ARCHIVE)
	touch downloaded

installed: downloaded
	tar xvzf $(SPLIT_SEQ_ARCHIVE)
	mkdir -p asdf
	touch rospack_nosubdirs
	ln -s ../split-sequence/split-sequence.asd asdf
	touch installed

clean:
	rm -rf split-sequence asdf
	rm -f $(SPLIT_SEQ_ARCHIVE)
	rm -f rospack_nosubdirs
	rm -f installed downloaded

