import {DataSourcePlugin} from '@grafana/data';
import {VistaLiveDataSource, VistaQuery} from './datasource';
export const plugin = new DataSourcePlugin<VistaLiveDataSource, VistaQuery>(VistaLiveDataSource);
