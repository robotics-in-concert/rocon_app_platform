/* 
   Jihoon Lee
    Date : 10.09.2012
 */

define(["dojo/_base/declare",
        "dojo/_base/lang",
        "dojo/dom-style",
        "dojo/on",
        "dijit/_WidgetBase",
        "dijit/form/Button",
        "dijit/form/SimpleTextarea",
        ],
function(declare,lang,domStyle,on,widgetbase,Button,Textarea)
{
    var AppLogView = declare("appmanager.AppLogView",[widgetbase],
        {
            log_topic_name : '/log',
            log_topic_type : 'std_msgs/String',
            logs : '',
            
            postCreate : function() {
                domStyle.set(this.domNode,'width','98%');
                domStyle.set(this.domNode,'position','relative');
                this.createTextarea();
                this.logSubscriber = new ros.Topic({
                    name : this.log_topic_name,
                    type : this.log_topic_type,
                    });

                this.createClearButton();

                ros.on('connection',lang.hitch(this,this.onConnect)); 
                ros.on('close',lang.hitch(this,this.onClose));
            },

            createClearButton : function() {
                this.button = new Button({label:"Clear", style:"float:right"});
                on(this.button,'click',lang.hitch(this,this.clear));
                this.domNode.appendChild(this.button.domNode);
            },

            clear : function(e) {
                this.textarea.innerHTML = "";
            },

            onConnect : function() {
                this.logSubscriber.subscribe(lang.hitch(this,this.listener));
            },

            onClose : function() {
                this.logSubscriber.unsubscribe();
            },

            createTextarea : function() {
                this.textarea = document.createElement('div'); 
                domStyle.set(this.textarea,'height','200px');
                domStyle.set(this.textarea,'width','100%');
                domStyle.set(this.textarea,'text-align','left');
                domStyle.set(this.textarea,'overflow','auto');
                domStyle.set(this.textarea,'border','1px solid grey');

                this.domNode.appendChild(this.textarea);
//                domStyle.set(this.textarea.domNode,'height','200px');
            },

            listener : function(msg) {
                var data = msg.data.split("\n").join("<br/>");
                if(data.charCodeAt(0) == 27) {
                    if(data.substring(1,4) =='[1m')
                        data = data.substring(4,data.length-3).bold();
                    // later more escape sequence will be suppported
                      
                }
                this.textarea.innerHTML += data;
                 
                this.textarea.scrollTop = this.textarea.scrollHeight;
//                domStyle.set(this.textarea,'scrollTop',this.t);

            },

        });
    return AppLogView;
}
);
