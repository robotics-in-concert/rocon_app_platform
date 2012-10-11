/* 
   Jihoon Lee
    Date : 10.04.2012
 */

define(["dojo/_base/declare",
        "dojo/_base/lang",
        "dijit/_WidgetBase",
        "dojo/dom-style",
        "dojo/on",
        "dojox/grid/DataGrid",
        "dojo/data/ItemFileWriteStore",
        "dijit/form/Button",
        "yujin_webtools/widgets/Loader",
        "dijit/Tooltip",
        ],
function(declare,lang,widgetbase,domStyle,on,DataGrid,Store,Button,Loader,Tooltip)
{
    var AppListView = declare("appmanager.AppListView",[widgetbase],
        {
            listapp_srv_name : '/list_apps',
            listapp_srv_type : '/appmanager_comms/GetAppList',

            current_app_list : [],


            postCreate : function() {
                Loader.loadCSS("dojox/grid/resources/Grid.css");
                Loader.loadCSS("dojox/grid/resources/claroGrid.css");
        
                this.updateAppListSrv = new ros.Service({
                    name : this.listapp_srv_name,
                    type: this.listapp_srv_type,
                });
                this.srvRequest = new ros.ServiceRequest({});

                this.createDataGrid();
//                this.createButton();

                ros.on('error',function(e) { console.log(e);});
                ros.on('connection',lang.hitch(this,this.onConnect)); 
                ros.on('close',lang.hitch(this,this.onClose));

                this.connect(this.grid,"onClick","onClick");
            },

            onConnect : function() {
                this.updateAppList();
                this.intervalInt = window.setInterval(lang.hitch(this,this.updateAppList),1000);
            },

            onClose : function() {
                window.clearInterval(this.intervalInt);
                var data = this.createData([]);
                this.currentStore.close();
                this.currentStore = new Store({data:data});
                this.grid.setStore(this.currentStore);

            },

            onClick : function(e) {
                e.selectedObject = this.current_app_list[e.rowIndex];
            },

            createDataGrid : function() {
                var data = this.createData([]); 

                var that = this;
                var layout = [ 
                               {name : "Name", field : "name",width:'70px'},
                               {name : "Status", field : "status",width:'70px'},
                             ];

                
                this.currentStore = new Store({data:data}); 

                this.grid = new DataGrid({
                    id: 'grid',
                    store : this.currentStore,
                    structure :layout,
                    rowSelector : '20px'
                    });
                domStyle.set(this.grid.domNode,"height","300px");
                domStyle.set(this.grid.domNode,"width","95%");
                domStyle.set(this.domNode,"margin","10px");

                this.domNode.appendChild(this.grid.domNode);
                this.grid.startup();
            },

            createButton : function() {
                this.button = new Button({label:"Get App lists"});
                this.connect(this.button,"onClick","updateAppList");
                this.domNode.appendChild(this.button.domNode);
            },

            updateAppList : function() {
                var that = this;
//                this.button.setAttribute('disabled',true);
                this.updateAppListSrv.callService(this.srvRequest,function(result) {
                    var data = that.createData(result.apps);
                    that.currentStore.close();
                    that.currentStore = new Store({data:data});
                    that.grid.setStore(that.currentStore);
//                    that.button.setAttribute('disabled',false);

//                    Tooltip.show("Updated",that.button.domNode);
//                    window.setTimeout(function() {  Tooltip.hide(that.button.domNode);  },1000);
                });
            },

            createData : function(app_descriptions) {
                var data = {
                    identifier : 'name',
                    items : [],
                };

                for( n in app_descriptions) { 
                    var app = app_descriptions[n];
                    var i = { 
                              name : app.name,
                              status : app.status,
                            };
                    data.items.push(i);
                }
                this.current_app_list = app_descriptions;
                return data;
            },
        });
    return AppListView;
}
);
