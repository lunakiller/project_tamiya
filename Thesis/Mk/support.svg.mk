-include $(MK_DIR)/paths.mk

SVG = $(wildcard *.svg)
PDF = $(patsubst %.svg, %.pdf, $(SVG))

DEST_DIR=../../fig/

DEST_PDF = $(addprefix $(DEST_DIR),$(PDF))

targets = pdf
copy_targets += copy_pdf

all: $(targets)

pdf: $(PDF)

%.pdf: %.svg
	$(INKSCAPE) -A $@ $<

clean:
	$(RM) $(PDF)

.PHONY: $(DEST_PDF)

copy: $(copy_targets)

copy_pdf: pdf $(DEST_PDF)

$(DEST_PDF): $(DEST_DIR)%.pdf: %.pdf
	@$(ECHO) Start copy $< to $@$(shell $(TEST) -L $@ && $(UNLINK) $@ && $(ECHO) ", but remove existing link in $(DEST_DIR) at first") 
	@$(CP) $< $@
