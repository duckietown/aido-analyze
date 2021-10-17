from setuptools import setup


def get_version(filename: str):
    import ast

    version = None
    with open(filename) as f:
        for line in f:
            if line.startswith("__version__"):
                version = ast.parse(line).body[0].value.s
                break
        else:
            raise ValueError("No version found in %r." % filename)
    if version is None:
        raise ValueError(filename)
    return version


version = get_version(filename="src/aido_analyze/__init__.py")

line = "daffy"

setup(
    name=f"aido-analyze-{line}",
    version=version,
    keywords="",
    package_dir={"": "src"},
    packages=["aido_analyze", "aido_analyze_tests"],
    install_requires=[
        f"aido-protocols-{line}",
        "zuper-ipce-z6",
        "zuper-commons-z6",
        "progressbar2",
        "procgraph-z6",
        f"duckietown-world-{line}",
        "PyGeometry-z6",
        "PyYAML",
        "numpy",
        "cbor2",
    ],
    entry_points={
        "console_scripts": [
            "aido-log-draw=aido_analyze.utils_drawing:aido_log_draw_main",
            "aido-log-video=aido_analyze.utils_video:aido_log_video_main",
            "aido-log-video-ui-image=aido_analyze.utils_video:aido_log_video_ui_image_main",
        ],
    },
)
