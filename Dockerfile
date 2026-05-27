FROM ghcr.io/ad-sdl/madsci:v0.8.0

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/pf400_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the PF400 plate handler robots"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p pf400_module

COPY ./src pf400_module/src
COPY ./README.md pf400_module/README.md
COPY ./pyproject.toml pf400_module/pyproject.toml

RUN --mount=type=cache,target=/root/.cache \
    uv pip install --python ${MADSCI_VENV}/bin/python -e ./pf400_module

CMD ["python", "pf400_module/src/pf400_rest_node.py"]

#########################################
