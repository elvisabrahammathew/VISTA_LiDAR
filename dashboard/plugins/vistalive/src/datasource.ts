import {DataFrame, DataSourceApi, DataSourceInstanceSettings, DataQuery, DataQueryRequest, DataQueryResponse, FieldType, LoadingState} from '@grafana/data';
import {getDataSourceSrv} from '@grafana/runtime';
import {defer, from, map, Observable, repeat, retry, Subscription, tap} from 'rxjs';
import {Freshness} from './freshness';

// Grafana's native StreamingDataFrame is a mutable internal buffer. Forwarding
// it after an empty result can enter the same-schema fast path with no previous
// frame (Grafana 13.2.2). Emit ordinary frames and detach values from that buffer.
export function snapshotFrames(frames: DataFrame[]): DataFrame[] {
  return frames.map(frame => ({
    name: frame.name,
    refId: frame.refId,
    meta: frame.meta ? {...frame.meta} : undefined,
    length: frame.length,
    fields: frame.fields.map(field => {
      const {values, state: _cachedState, ...definition} = field;
      return {
        ...definition,
        config: {...field.config},
        labels: field.labels ? {...field.labels} : undefined,
        values: Array.from(values),
      };
    }),
  }));
}

export interface VistaQuery extends DataQuery {
  channel?: string;
  queryType?: string;
  buffer?: number;
  filter?: {fields?: string[]};
}

export class VistaLiveDataSource extends DataSourceApi<VistaQuery> {
  constructor(settings: DataSourceInstanceSettings) { super(settings); }
  query(request: DataQueryRequest<VistaQuery>): Observable<DataQueryResponse> {
    return new Observable(subscriber => {
      const lifetime = new Subscription();
      const freshness = new Freshness();
      let payload: DataQueryResponse['data'] = [];
      let shown = false;
      let closed = false;
      let dataConnected = false;
      const emit = (force = false) => {
        const live = dataConnected && freshness.live(Date.now());
        if (force || live !== shown) {
          // No response key: replace the panel result, including with [] on
          // expiry. Do not append an empty frame to Grafana's old live buffer.
          subscriber.next({data: live ? payload : [], state: LoadingState.Streaming});
          shown = live;
        }
      };
      emit(true);
      const timer = setInterval(() => emit(), 250);
      lifetime.add(() => clearInterval(timer));
      getDataSourceSrv().get('grafana').then(builtin => {
        if (closed) { return; }
        const builtinRequest = (targets: VistaQuery[]): DataQueryRequest<VistaQuery> => ({
          ...request,
          targets: targets.map(target => ({...target, datasource: {type: 'grafana', uid: 'grafana'}})),
        });
        // Recover both errors and normal completion independently. Live can
        // complete on disconnect; retry alone would leave the panel empty.
        // Timers and both streams are always cancelled when the panel closes.
        const connect = (heartbeat: boolean) => {
          if (closed) { return; }
          const targets: VistaQuery[] = heartbeat ? [{
            refId: '__VISTA_HEARTBEAT', queryType: 'measurements', buffer: 10000,
            channel: 'stream/vista/grafana_bridge_health', filter: {fields: ['time']},
          }] : request.targets.filter(target => !target.hide);
          if (!targets.length) { return; }
          const disconnected = () => {
            if (heartbeat) { freshness.reset(); } else { dataConnected = false; }
            emit(true);
          };
          const stream = defer(() => from(builtin.query(builtinRequest(targets)))).pipe(
            map(response => {
              if (response.error || response.errors?.length) {
                throw response.error ?? response.errors![0];
              }
              return response;
            }),
            tap({error: disconnected, complete: disconnected}),
            retry({delay: 1000, resetOnSuccess: true}),
            repeat({delay: 1000}),
          );
          const subscription = stream.subscribe({
            next: response => {
              if (heartbeat) {
                for (const frame of response.data) {
                  for (const field of frame.fields ?? []) {
                    if (field.type === FieldType.time) {
                      for (const value of field.values) { freshness.observe(Number(value)); }
                    }
                  }
                }
              } else {
                payload = snapshotFrames(response.data);
                dataConnected = true;
              }
              emit(!heartbeat);
            },
          });
          lifetime.add(subscription);
        };
        connect(true);
        connect(false);
      }).catch(error => { if (!closed) { subscriber.error(error); } });
      return () => { closed = true; lifetime.unsubscribe(); payload = []; };
    });
  }
  async testDatasource() {
    await getDataSourceSrv().get('grafana');
    return {status: 'success', message: 'Grafana Live wrapper ready. Data appears only while the VISTA producer heartbeat is fresh.'};
  }
}
