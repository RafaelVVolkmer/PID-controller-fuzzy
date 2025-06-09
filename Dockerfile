# syntax=docker/dockerfile:1.4

# ------------------------------------------------------------
# Stage 1: Builder
# ------------------------------------------------------------
FROM alpine:3.18 AS builder

RUN apk add --no-cache \
        bash \
        build-base \
        cmake

WORKDIR /app

COPY . .

RUN chmod +x build.sh

ARG BUILD_MODE=release

RUN ./build.sh $BUILD_MODE

# ------------------------------------------------------------
# Stage 2: Runtime
# ------------------------------------------------------------
FROM alpine:3.18 AS runtime

RUN apk add --no-cache libstdc++

COPY --from=builder /app/build/pid_app /usr/local/bin/

WORKDIR /usr/local/bin

ENTRYPOINT ["pid_app"]

# ------------------------------------------------------------
# Stage 3: Export
# ------------------------------------------------------------
FROM scratch AS export

COPY --from=builder /app/build /out
