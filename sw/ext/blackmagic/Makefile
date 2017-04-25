# Hey Emacs, this is a -*- makefile -*-

BLACKSPHERE = https://github.com/blacksphere/blackmagic/releases/download/v1.6
BLACKMAGIC = blackmagic-native-v1.6.bin

Q=@

all: $(BLACKMAGIC) dfu.py
	$(Q)./stm32_mem.py ./$(BLACKMAGIC)

$(BLACKMAGIC):
	@echo "Downloading latest BMP firmware"
	wget -O ./$(BLACKMAGIC) $(BLACKSPHERE)/$(BLACKMAGIC)

dfu.py:
	ln -s -f ../../tools/dfu/dfu.py dfu.py

clean:
	$(Q) rm -rf ./$(BLACKMAGIC)
