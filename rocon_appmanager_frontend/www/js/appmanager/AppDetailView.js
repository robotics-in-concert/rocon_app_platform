/* 
   Jihoon Lee
    Date : 10.09.2012
 */

define(["dojo/_base/declare",
        "dojo/_base/lang",
        "dijit/_WidgetBase",
        "dojo/dom-style",
        "dijit/form/Button",
        "yujin_webtools/widgets/Loader",
        "dijit/Tooltip",
        ],
function(declare,lang,widgetbase,domStyle,Button,Loader,Tooltip)
{
    var AppDetailView = declare("appmanager.AppDetailView",[widgetbase],
        {
            start_app_srv_name : '/start_app',
            start_app_srv_type : '/appmanager_msgs/StartApp',

            stop_app_srv_name : '/stop_app',
            stop_app_srv_type : '/appmanager_msgs/StopApp',

            currentData : null,
            
            postCreate: function() {
               this.createButton(); 
               this.startAppSrv = new ros.Service({
                   name : this.start_app_srv_name,
                   type: this.start_app_srv_type,
               });

               this.stopAppSrv = new ros.Service({
                   name : this.stop_app_srv_name,
                   type: this.stop_app_srv_type,
               });

               ros.on('connection',lang.hitch(this,this.onConnect)); 
               ros.on('close',lang.hitch(this,this.onClose));

               var div = document.createElement('div');
               var strong = document.createElement('strong');
               strong.innerHTML = 'No Connection';
               div.appendChild(strong);
               this.noConnectDiv = div;
               this.domNode.appendChild(this.noConnectDiv);
            },

            onConnect : function() {
                this.domNode.removeChild(this.noConnectDiv);
            },

            onClose : function() {
                this.domNode.innerHTML = "";
                this.domNode.appendChild(this.noConnectDiv);
            },

            createButton : function() {
                this.start_button = new Button({label:"Start app"});
                this.stop_button = new Button({label:"Stop app"});
                this.connect(this.start_button,"onClick","startApp");
                this.connect(this.stop_button,"onClick","stopApp");
            },

            display: function(data) {
                if(data == undefined || data == null) { 
                    if(this.currentDisplay != null) {
                        this.domNode.removeChild(this.currentDisplay); 
                        this.currentDisplay = null;
                    }
                    return;
                }
                if(this.currentDisplay != null) 
                    this.domNode.removeChild(this.currentDisplay); 

                var div = document.createElement('div');
                for(key in data) {
                    var element = this.addElement(key,data[key]);
                    div.appendChild(element);
                }

                if(data.status == 'Ready') {
                    this.start_button.setAttribute('disabled',false);
                }
                else
                    this.start_button.setAttribute('disabled',true);

                if(data.status == 'Running')
                    this.stop_button.setAttribute('disabled',false);
                else
                    this.stop_button.setAttribute('disabled',true);

                div.appendChild(this.start_button.domNode);
                div.appendChild(this.stop_button.domNode);
                this.currentDisplay = div;
                this.currentData = data;
                this.domNode.appendChild(this.currentDisplay);
            },

            startApp : function(e) {
                this.start_button.setAttribute('disabled',true);

                this.srvRequest = new ros.ServiceRequest({name:this.currentData.name});
                var that = this;
                this.startAppSrv.callService(this.srvRequest, lang.hitch(this,this.processStartAppResponse) );
            },

            processStartAppResponse : function(result) {
                var message = '';
                var that = this;

                // if the app is started 
                if(result.started) {
                    message = result.message
                    this.stop_button.setAttribute('disabled',false);
                    this.start_button.setAttribute('disabled',true);
                }
                else {
                    message = result.message
                    this.start_button.setAttribute('disabled',false);
                }

                Tooltip.show(result.message,that.start_button.domNode,["below"]);
                window.setTimeout(function() {  Tooltip.hide(that.start_button.domNode);  },1000);
            },

            stopApp : function(e) {
                this.stop_button.setAttribute('disabled',true);

                this.srvRequest = new ros.ServiceRequest({name:this.currentData.name});
                var that = this;
                this.stopAppSrv.callService(this.srvRequest, lang.hitch(this,this.processStopAppResponse) );
            },

            processStopAppResponse : function(result) {
                var that = this;

                // if the app is started 
                if(result.stopped) {
                    this.stop_button.setAttribute('disabled',true);
                    this.start_button.setAttribute('disabled',false);
                }
                else {
                    this.stop_button.setAttribute('disabled',false);
                }

                Tooltip.show(result.message,that.stop_button.domNode,["below"]);
                window.setTimeout(function() {  Tooltip.hide(that.stop_button.domNode);  },1000);
            },

            addElement : function(key,data)
            {
                var div = document.createElement('div');
                // key
                var strong = document.createElement('strong');
                strong.innerHTML = key + ' : ';
                var p = document.createElement('p');
                p.innerHTML = data;

                div.appendChild(strong);
                div.appendChild(p);

                return div;
            },
        });
    return AppDetailView;
}
);
