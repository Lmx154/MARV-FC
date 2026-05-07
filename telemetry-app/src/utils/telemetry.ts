import { useMemo } from "react";

import type { AppState, TelemetryField, TelemetryLookup } from "../types";

export function findValue(state: AppState | null, group: string, parameter: string, fallback: string) {
  return state?.hilink.fields.find((field) => field.group === group && field.parameter === parameter)?.value ?? fallback;
}

export function useTelemetryLookup(state: AppState | null): TelemetryLookup {
  const fieldMap = useMemo(() => {
    const fields = new Map<string, TelemetryField>();
    for (const field of state?.hilink.fields ?? []) {
      fields.set(`${field.group}.${field.parameter}`, field);
    }
    return fields;
  }, [state]);

  return {
    field: (group: string, parameter: string, fallback = "--") =>
      fieldMap.get(`${group}.${parameter}`)?.value ?? fallback,
    fieldUnit: (group: string, parameter: string) => fieldMap.get(`${group}.${parameter}`)?.unit ?? "",
  };
}
