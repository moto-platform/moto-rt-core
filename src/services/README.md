# src/services

Shared services the feature modules talk through: signal pool, com (generated pack/unpack + E2E from `gen/c/rt_core/`), diag (UDS server/client on top of the ISO-TP core), timebase, log. Features never include each other, only services. Empty for now.
