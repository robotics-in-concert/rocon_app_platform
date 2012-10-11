/* 
   Jihoon Lee
    Date : 10.09.2012
 */

define(["dojo/_base/declare",
        "dojo/_base/lang",
        "dojo/dom-style",
        "dijit/_WidgetBase",
        "dijit/form/Button",
        "dijit/form/SimpleTextarea",
        ],
function(declare,lang,domStyle,widgetbase,Button,Textarea)
{
    var AppLogView = declare("appmanager.AppLogView",[widgetbase],
        {
            log_topic_name : '/log',
            log_topic_type : '/std_msgs/String',
            logs : '',
            
            postCreate : function() {
                domStyle.set(this.domNode,'width','98%');
                domStyle.set(this.domNode,'position','relative');
                this.createTextarea();
                this.logSubscriber = new ros.Topic({
                    name : this.log_topic_name,
                    type : this.log_topic_type,
                    });

                ros.on('connection',lang.hitch(this,this.onConnect)); 
                ros.on('close',lang.hitch(this,this.onClose));
            },

            onConnect : function() {
                this.logSubscriber.subscribe(lang.hitch(this,this.listener));
            },

            onClose : function() {
                this.logSubscriber.unsubscribe();
            },

            createTextarea : function() {
                this.textarea = new Textarea({
                    value : '',
                    style : "width:100%; margin-bottom:2pt;",
                    readOnly : true,
                    });
                this.domNode.appendChild(this.textarea.domNode);
                domStyle.set(this.textarea.domNode,'height','200px');
            },

            listener : function(msg) {
                this.logs += msg + '\n';
                this.textarea.setAttribute('value',this.logs);  
            },

        });
    return AppLogView;
}
);
