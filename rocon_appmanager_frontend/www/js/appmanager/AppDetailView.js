/* 
   Jihoon Lee
    Date : 10.09.2012
 */

define(["dojo/_base/declare",
        "dijit/_WidgetBase",
        "dojo/dom-style",
        "dijit/form/Button",
        "yujin_webtools/widgets/Loader",
        "dijit/Tooltip",
        ],
function(declare,widgetbase,domStyle,Button,Loader,Tooltip)
{
    var AppDetailView = declare("appmanager.AppDetailView",[widgetbase],
        {
            start_app_srv_name : '/start_app',
            currentData : null,
            
            postCreate: function() {
               this.createButton(); 
            },

            createButton : function() {
                this.button = new Button({label:"Start app"});
                this.connect(this.button,"onClick","startApp");
            },

            display: function(data) {
                if(data == undefined || data == null)
                    return;
                if(this.currentDisplay != null) 
                    this.domNode.removeChild(this.currentDisplay); 

                var div = document.createElement('div');
                for(key in data) {
                    var element = this.addElement(key,data[key]);
                    div.appendChild(element);
//                    var br = document.createElement('br');
//                    div.appendChild(br);
                }


                div.appendChild(this.button.domNode);
                this.currentDisplay = div;
                this.currentData = data;
                this.domNode.appendChild(this.currentDisplay);
            },

            startApp : function(e) {
                console.log(this.currentData);
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
