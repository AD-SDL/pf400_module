FROM ghcr.io/ad-sdl/wei

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/pf400_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the PF400 plate handler robots"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p pf400_module

COPY ./pf400_driver pf400_module/pf400_driver
COPY ./scripts pf400_module/scripts
COPY ./README.md pf400_module/README.md
COPY ./pyproject.toml pf400_module/pyproject.toml
COPY ./tests pf400_module/tests

RUN --mount=type=cache,target=/root/.cache \
    pip install -e ./pf400_module

CMD ["python", "pf400_module/scripts/pf400_rest_node.py"]

#########################################
