/* 
   Jihoon Lee
    Date : 10.09.2012
 */

define(["dojo/_base/declare",
        "dojo/_base/lang",
        "dijit/_WidgetBase",
        "dijit/form/Button",
        "dijit/form/Textarea",
        ],
function(declare,lang,widgetbase,Button,Textarea)
{
    var AppLogView = declare("appmanager.AppLogView",[widgetbase],
        {
            log_topic_name : '/log',
            log_topic_type : '/std_msgs/String',
            logs : '',
            
            postCreate : function() {
                this.createTextarea();
                this.logSubscriber = new ros.Topic({
                    name : this.log_topic_name,
                    type : this.log_topic_type,
                    });

//                this.logSubscriber.subscribe(lang.hitch(this,this.listener));
            },

            createTextarea : function() {
                this.textarea = new Textarea({
                    value : 'No log yet',
                    });
                this.domNode.append(this.textarea.domNode);
            },

            listener : function(msg) {
                this.logs += msg + '\n';
                this.textarea.setAttribute('value',this.logs);  
            },

        });
    return AppLogView;
}
);
