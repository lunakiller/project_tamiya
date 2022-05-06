#SUBDIRS+=fig
#SUBDIRS+=dia
#SUBDIRS+=eps
SUBDIRS+=pic
SUBDIRS+=pdf
SUBDIRS+=tikz
#SUBDIRS+=mat
SUBDIRS+=svg

.PHONY: subdirs $(SUBDIRS) copy clean 

ifeq ($(origin MAKECMDGOALS), undefined)
MAKECMDGOALS = all
endif

subdirs: $(SUBDIRS)

$(SUBDIRS):
	echo "Generate $(MAKECMDGOALS) in $@"
	$(MAKE) -C $@ $(MAKECMDGOALS)

generate: subdirs

clean: subdirs

copy: subdirs

all: generate copy
