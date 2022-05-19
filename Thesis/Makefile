
SUPPORTDIR = support

NAME = main
TARGET = main.pdf

single:
	$(MAKE) pdf

full:
	$(MAKE) pdf
	$(MAKE) bib
	$(MAKE) pdf
	$(MAKE) pdf

x:
	$(MAKE) cleanall
	$(MAKE) full
	
#PRE_BUILD=xslt

MK_DIR=Mk
include $(MK_DIR)/common.mk


