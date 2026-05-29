FROM ghcr.io/ad-sdl/madsci:v0.8.0

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/pf400_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the PF400 plate handler robots"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

ARG USER_ID=9999
ARG GROUP_ID=9999

COPY ./src /home/madsci/pf400_module/src
COPY ./README.md /home/madsci/pf400_module/README.md
COPY ./pyproject.toml /home/madsci/pf400_module/pyproject.toml

RUN --mount=type=cache,target=/root/.cache \
    uv pip install --python ${MADSCI_VENV}/bin/python -e /home/madsci/pf400_module && \
    chown -R ${USER_ID}:${GROUP_ID} /home/madsci/pf400_module

CMD ["python", "-m", "pf400_rest_node"]

#########################################
