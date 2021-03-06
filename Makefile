
all:


bump-upload:
	$(MAKE) bump
	$(MAKE) upload

docs:

bump: # v2
	bumpversion patch
	git push --tags
	git push

upload: # v3
	dts build_utils check-not-dirty
	dts build_utils check-tagged
	dts build_utils check-need-upload --package aido-analyze-daffy make upload-do

upload-do:
	rm -f dist/*
	rm -rf src/*.egg-info
	python3 setup.py sdist
	twine upload --skip-existing --verbose dist/*

log=ETHZ_autolab_technical_track-sc0-0/log.gs2.cbor
banner=banner1.png

test: test-ui test-ego test-draw

test-ui:
	aido-log-video-ui-image  --gslog $(log) --out out/ui-image.mp4

test-ego:
	aido-log-video --gslog $(log) --robot ego --out out/test1.mp4 --banner $(banner) --banner-bottom $(banner)

test-draw:
	aido-log-draw --gslog $(log) --robot ego --outdir out/test1
