FROM ghcr.io/ad-sdl/wei

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/ot2_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the Opentrons OT2 LiquidHandling robots"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p ot2_module

COPY ./ot2_driver ot2_module/ot2_driver
COPY ./src ot2_module/src
COPY ./README.md ot2_module/README.md
COPY ./pyproject.toml ot2_module/pyproject.toml
COPY ./tests ot2_module/tests

RUN --mount=type=cache,target=/root/.cache \
    pip install -e ./ot2_module

CMD ["python", "ot2_module/src/ot2_rest_node.py"]

#########################################
