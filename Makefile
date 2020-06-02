
all:


bump-upload:
	$(MAKE) bump
	$(MAKE) upload

bump:
	bumpversion patch
	git push --tags
	git push

upload:
	rm -f dist/*
	rm -rf src/*.egg-info
	python setup.py sdist
	twine upload dist/*


