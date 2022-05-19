-include $(MK_DIR)/paths.mk

TIKZ = $(wildcard *.tex)
PDF = $(patsubst %.tex, %.pdf, $(TIKZ))
FILES = $(patsubst %.tex, %.log, $(TIKZ))
FILES += $(patsubst %.tex, %.aux, $(TIKZ))

DEST_DIR=../../fig/

DEST_PDF = $(addprefix $(DEST_DIR),$(PDF))

targets = pdf
copy_targets += copy_pdf

all: $(targets)

pdf: $(PDF)

%.pdf: %.tex
	$(LATEXPDF) $<

clean:
	$(RM) $(PDF) $(FILES)

.PHONY: $(DEST_PDF)

copy: $(copy_targets)

copy_pdf: pdf $(DEST_PDF)

$(DEST_PDF): $(DEST_DIR)%.pdf: %.pdf
	@$(ECHO) Start copy $< to $@$(shell $(TEST) -L $@ && $(UNLINK) $@ && $(ECHO) ", but remove existing link in $(DEST_DIR) at first") 
	@$(CP) $< $@
